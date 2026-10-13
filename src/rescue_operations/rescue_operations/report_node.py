"""ROS 2 경계: 조정기 RescueReport(IF-072) -> 저장 -> ReportReceipt(IF-073).

``report_store.RescueReportStore`` 에 넘기고, 돌아온 영수증(PERSISTED 또는 REJECTED 와 이유)을 그대로
발행한다. 판정·재시도 정책은 없다: 재전송은 조정기가 하고, 같은 보고는 장부가 한 번만 저장한다.
DB 파일은 site_state DB(db_writer)와 다른 파일이다(파일당 쓰는 프로세스 하나).

파라미터는 모두 필수이고 기본값이 없다(AGENTS 8절). ROS 패키지는 main() 안에서 import 한다.
QoS 는 SDD v0.1 IF-072·073: RELIABLE / VOLATILE / KEEP_LAST 10.
"""
import time
from uuid import uuid4

from .report_store import RescueReportStore, receipt_fields
from .ros_node import git_sha
from .schema import OperationsError

PARAMS = {'db_path': 'STRING', 'reports_topic': 'STRING', 'receipts_topic': 'STRING',
          'envelope_schema_version': 'STRING', 'publisher_id': 'STRING'}
QOS_DEPTH = 10


def main(args=None):
    import rclpy
    from rclpy.exceptions import ParameterUninitializedException
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from rescue_interfaces.msg import ReportReceipt, RescueReport
    from rosidl_runtime_py import message_to_ordereddict

    rclpy.init(args=args)
    node = Node('rescue_report_node')
    log = node.get_logger()
    values = {}
    for name, kind in PARAMS.items():
        node.declare_parameter(name, getattr(Parameter.Type, kind))
        try:
            values[name] = node.get_parameter(name).value
        except ParameterUninitializedException:
            values[name] = None
    missing = sorted(name for name, value in values.items() if value is None)
    if missing:
        log.fatal(f'missing parameters: {missing}')
        raise SystemExit(2)
    if values['reports_topic'] == values['receipts_topic']:
        log.fatal('reports_topic and receipts_topic must differ')
        raise SystemExit(2)
    try:
        store = RescueReportStore(values['db_path'])
    except OperationsError as exc:
        log.fatal(f'cannot open report store: {exc.detail_code}: {exc.detail}')
        raise SystemExit(2) from exc
    boot_id = str(uuid4())
    log.info(f'rescue_report_node git={git_sha()} boot={boot_id} db={store.path} '
             f'reports={values["reports_topic"]} receipts={values["receipts_topic"]}')
    qos = QoSProfile(depth=QOS_DEPTH, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)
    publisher = node.create_publisher(ReportReceipt, values['receipts_topic'], qos)
    sequence = [0]

    def on_report(msg):
        receipt = store.save(message_to_ordereddict(msg))
        out = ReportReceipt()
        sequence[0] += 1
        out.envelope.schema_version = values['envelope_schema_version']
        out.envelope.publisher_id = values['publisher_id']
        out.envelope.publisher_boot_id = boot_id
        out.envelope.sequence = sequence[0]
        out.envelope.published_at_ns = time.time_ns()
        for name, value in receipt_fields(receipt).items():
            setattr(out, name, value)
        publisher.publish(out)
        line = (f'REPORT {receipt.report_id or "?"} target={msg.target_id} records={len(msg.record_ids)} '
                f'-> {receipt.state}')
        if receipt.state == 'PERSISTED':
            log.info(f'{line} receipt={receipt.receipt_id}')
        else:
            log.error(f'{line} reason={receipt.reason_code}')

    node.create_subscription(RescueReport, values['reports_topic'], on_report, qos)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        log.info('rescue_report_node stopping')
    finally:
        store.close()
        node.destroy_node()
        rclpy.try_shutdown()

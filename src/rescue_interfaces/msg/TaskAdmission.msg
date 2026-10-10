# SDD v0.1 message_schemas.TaskAdmission (docs/design/sdd-node-contracts-v0.1.json).
# Fields come from that JSON; change the JSON first. tests/test_interfaces_schema.py checks it.
# goal accepted=false의 상세 이유 및 수락 장부의 근거.
Envelope envelope
string admission_id
string command_id
string task_id
string robot_id
string robot_boot_id
string authority_epoch
uint64 generation
bool accepted
string reason_code
int64 observed_at_ns

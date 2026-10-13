"""SIGTERM shutdown race of the operations nodes (official rehearsal 2026-10-13; coordinator #154, perception #222)."""


def spin_until_shutdown(spin, context_ok, shutdown_error):
    """Run ``spin``; a ``shutdown_error`` raised after the context closed is the shutdown race, a normal stop.

    On SIGTERM rclpy shuts the context down, and a spin caught between callbacks raises
    RCLError("the given context is not valid") instead of ExternalShutdownException (official rehearsal on
    main 2098ca2: db_writer 1 of 3 runs ended with a traceback and exit 1). The same error while the context
    is valid, and every other error, is raised.
    """
    try:
        spin()
    except shutdown_error:
        if context_ok():
            raise

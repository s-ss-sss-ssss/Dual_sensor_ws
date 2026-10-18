"""CLI and ROS lifecycle for independent fixed-matrix validation."""
import argparse
import csv
import json
import signal
import sys

from .consistency_validation import load_validation_config, make_validation_plan
from .validation_collect import ValidationCollector, ValidationStore, compare_runs, reanalyze


def main(argv=None):
    parser = argparse.ArgumentParser(description='Independent fixed-calibration validation; no fitting')
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument('--config')
    source.add_argument('--reanalyze', metavar='RUN_DIRECTORY')
    source.add_argument('--compare', nargs='+', metavar='RUN_DIRECTORY')
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--check', action='store_true')
    mode.add_argument('--dry-run', action='store_true')
    args, extra = parser.parse_known_args(argv)
    if any(not a.startswith(('__name:=', '__log:=', '__ns:=')) for a in extra):
        parser.error('unrecognized arguments: ' + ' '.join(extra))
    if (args.check or args.dry_run) and not args.config:
        parser.error('--check/--dry-run require --config')
    if args.compare:
        rows = compare_runs(args.compare)
        if rows:
            writer = csv.DictWriter(sys.stdout, fieldnames=list(rows[0]))
            writer.writeheader()
            for row in rows:
                writer.writerow({k: json.dumps(v, ensure_ascii=False, sort_keys=True) if isinstance(v, (list, dict)) else v
                                 for k, v in row.items()})
        return
    if args.reanalyze:
        print(reanalyze(args.reanalyze))
        return
    config = load_validation_config(args.config)
    plan = make_validation_plan(config)
    print('Configuration valid: %s independent validation trials; fixed reference %s; %s C, %s.' %
          (len(plan), config['reference_id'], config['temperature_c'], config['environment_label']))
    if args.check:
        return
    if args.dry_run:
        print('STARTUP: all_off -> settle -> startup check; retain frozen reference.')
        for t in plan:
            print('%s: %s; pre/off -> settle %.1fs -> capture %.1fs; on -> settle -> capture; post/off -> settle -> capture' %
                  (t['trial_id'], t['commands_v'], config['settle_s'], config['capture_s']))
        print('all_off confirmed -> fixed T evaluation, MaPS X, long/short reports; no fitting.')
        return
    import rospy
    from .consistency_ros import RosBackend
    rospy.init_node('consistency_validation', disable_signals=True)
    store = ValidationStore(config)
    backend = None
    handlers = {}
    try:
        backend = RosBackend(config, store)
        def stop(_signum, _frame):
            backend.cancelled = True
        for signum in (signal.SIGINT, signal.SIGTERM):
            handlers[signum] = signal.signal(signum, stop)
        print('Run directory: %s' % store.path, flush=True)
        ValidationCollector(config, backend, store).run()
        print('Validation results: %s' % store.path)
    except KeyboardInterrupt:
        raise SystemExit(130)
    finally:
        if backend is not None:
            for subscriber in backend.subscribers:
                subscriber.unregister()
            with backend.lock:
                backend.closed = True
                store.close()
        else:
            store.close()
        for signum, handler in handlers.items():
            signal.signal(signum, handler)
        rospy.signal_shutdown('validation finished')

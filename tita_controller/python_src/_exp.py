#!/usr/bin/env python3
"""Shared experiment-selection helper for TITA plotting scripts."""
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent


def select_experiment(base: Path, exp_arg: int | None = None) -> tuple[Path, Path]:
    """Return (log_dir, out_dir) for the chosen robot_logs_N experiment.

    Scans base/ for robot_logs_N subfolders. If exp_arg is given, uses that
    number directly. Otherwise lists available experiments and prompts the user.
    """
    folders = sorted(
        [d for d in base.iterdir() if d.is_dir() and d.name.startswith('robot_logs_')],
        key=lambda d: int(d.name.split('_')[-1])
    ) if base.exists() else []

    if not folders:
        print(f'  [warn] No experiment folders found under {base}. Using {base} directly.')
        return base, SCRIPT_DIR.parent / 'images'

    if exp_arg is not None:
        log_dir = base / f'robot_logs_{exp_arg}'
        if not log_dir.exists():
            print(f'  [warn] Experiment {exp_arg} not found, falling back to last.')
            log_dir = folders[-1]
    else:
        print('\nAvailable experiments:')
        for d in folders:
            n_files = sum(1 for _ in d.iterdir())
            print(f'  [{d.name.split("_")[-1]}] {d.name}  ({n_files} files)')
        print(f'  [last] = {folders[-1].name}')
        raw = input('\nExperiment number (Enter = last): ').strip()
        if raw == '' or raw.lower() == 'last':
            log_dir = folders[-1]
        else:
            log_dir = base / f'robot_logs_{raw}'
            if not log_dir.exists():
                print(f'  [warn] {log_dir} not found, using last.')
                log_dir = folders[-1]

    return log_dir, SCRIPT_DIR.parent / 'images'
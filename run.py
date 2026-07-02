#!/usr/bin/env python3
"""Launch the SKYFUSE server: python run.py [--port 8777] [--seed 42]"""
import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from skyfuse.server import main  # noqa: E402

if __name__ == '__main__':
    ap = argparse.ArgumentParser(description='SKYFUSE track fusion server')
    ap.add_argument('--port', type=int, default=8777)
    ap.add_argument('--seed', type=int, default=None,
                    help='RNG seed for a reproducible scenario')
    args = ap.parse_args()
    print(f'SKYFUSE up — open http://localhost:{args.port}')
    main(port=args.port, seed=args.seed)

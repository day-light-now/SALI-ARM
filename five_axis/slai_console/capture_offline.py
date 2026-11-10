"""Six-page screenshot entrypoint that hard-rejects physical serial opens."""
import sys
from unittest.mock import patch


def main():
    from .app import main as run
    sys.argv=[sys.argv[0],'--smoke']
    with patch('serial.Serial',side_effect=AssertionError('Screenshot run forbids real serial')):
        return run()


if __name__=='__main__':
    raise SystemExit(main())

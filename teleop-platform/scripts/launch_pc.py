from pathlib import Path
import sys

root=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(root/'src'),str(root/'upstream/dobot-sdk')]
from e6_phosphobot.pc import main
from typer import Exit
try:
    main()
except Exit as exc:
    raise SystemExit(exc.exit_code)

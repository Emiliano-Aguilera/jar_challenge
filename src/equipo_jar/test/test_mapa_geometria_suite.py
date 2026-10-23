"""Run shared-map geometry checks through colcon."""
import os
from pathlib import Path
import subprocess
import sys


def test_map_coordinate_suite():
    root = Path(__file__).resolve().parents[3]
    suite = root / 'pruebas' / 'test_mapa_coordenadas.py'
    assert suite.is_file(), 'Run map tests from the full source checkout'
    environment = dict(os.environ)
    environment['PYTHONPATH'] = (
        str(root / 'src' / 'equipo_jar') + os.pathsep
        + environment.get('PYTHONPATH', ''))
    result = subprocess.run(
        [sys.executable, '-m', 'pytest', str(suite), '-q', '--tb=short'],
        cwd=root, env=environment, capture_output=True, text=True, timeout=30)
    assert result.returncode == 0, result.stdout + result.stderr

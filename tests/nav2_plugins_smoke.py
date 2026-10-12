"""Check startup refusal for missing plugins and old closed-loop-only shims."""
import importlib.util
import os
from pathlib import Path
import tempfile

from ament_index_python.packages import get_package_share_directory, get_package_prefix

project = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('plugin_check', project/'go2_3d/check_nav2_plugins.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
if os.environ.get('GO2_NAV2_SOURCE'):
    source = Path(os.environ['GO2_NAV2_SOURCE'])
    module.get_package_share_directory = lambda package: str(source/package)
else:
    source = None
with tempfile.TemporaryDirectory() as directory:
    prefix = Path(directory)
    module.get_package_prefix = lambda package: str(prefix)
    relative = Path('nav2_rotation_shim_controller/nav2_rotation_shim_controller.hpp')
    if source:
        content = (source/'nav2_rotation_shim_controller/include'/relative).read_text()
    else:
        installed = Path(get_package_prefix('nav2_rotation_shim_controller'))/'include'
        content = next(path.read_text() for path in (installed/relative,
                       installed/'nav2_rotation_shim_controller'/relative) if path.is_file())
    header = prefix/'include'/relative
    header.parent.mkdir(parents=True)
    header.write_text(content)
    module.check()
    header.write_text(content.replace('closed_loop_', 'legacy_feedback_'))
    try:
        module.check()
    except RuntimeError as error:
        assert 'open-loop rotation support' in str(error) and 'build.sh' in str(error)
    else:
        raise AssertionError('Old shim silently ignored closed_loop=false')
    module.get_package_share_directory = lambda package: str(prefix/'missing'/package)
    try:
        module.check()
    except RuntimeError as error:
        assert 'DenoiseLayer' in str(error) and 'build.sh' in str(error)
    else:
        raise AssertionError('Missing Nav2 image accepted')
print('PASS plugin preflight: real official XML/header; supported image accepted, missing/closed-loop-only images rejected before launch')

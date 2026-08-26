import importlib.util
from pathlib import Path

import pytest


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]


def load_launch(package):
    path = REPOSITORY_ROOT / package / 'launch' / f'{package}_launch.py'
    if package == 'fastlio2':
        path = REPOSITORY_ROOT / package / 'launch' / 'lio_launch.py'
    spec = importlib.util.spec_from_file_location(f'{package}_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_mapping_has_only_pgo_as_global_tf_owner():
    lio = load_launch('fastlio2')
    pgo = load_launch('pgo')
    localizer = load_launch('localizer')

    assert lio.profile_owner('mapping') == 'none'
    assert pgo.profile_owner('mapping') == 'pgo'
    with pytest.raises(RuntimeError):
        localizer.profile_owner('mapping')


def test_localization_has_only_localizer_as_global_tf_owner():
    lio = load_launch('fastlio2')
    pgo = load_launch('pgo')
    localizer = load_launch('localizer')

    assert lio.profile_owner('localization') == 'none'
    assert localizer.profile_owner('localization') == 'localizer'
    with pytest.raises(RuntimeError):
        pgo.profile_owner('localization')


def test_online_launches_reject_maintenance_profile():
    for package in ('fastlio2', 'pgo', 'localizer'):
        with pytest.raises(RuntimeError):
            load_launch(package).profile_owner('maintenance')

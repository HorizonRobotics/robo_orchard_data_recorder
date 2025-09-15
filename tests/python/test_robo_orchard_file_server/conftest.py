# Project RoboOrchard
#
# Copyright (c) 2024-2025 Horizon Robotics. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#       http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or
# implied. See the License for the specific language governing
# permissions and limitations under the License.

import tempfile
from pathlib import Path

import pytest
from fastapi.testclient import TestClient
from robo_orchard_file_server.simple_file_server import app


@pytest.fixture(scope="session")
def test_directory() -> Path:
    """Creates a temporary directory with files for testing."""
    with tempfile.TemporaryDirectory() as tmpdir:
        base_path = Path(tmpdir)
        (base_path / "test.txt").write_text("This is a test file.")
        subdir = base_path / "subdir"
        subdir.mkdir()
        (subdir / "nested.txt").write_text("Nested content.")
        (base_path / "empty.txt").touch()
        yield base_path


@pytest.fixture
def patched_app_state(test_directory: Path, monkeypatch):
    """A fixture that correctly patches the application's BASE_DIR state."""
    # Explicitly import the module where BASE_DIR is defined.
    # We give it an alias to make the code clear.
    from robo_orchard_file_server import simple_file_server as server_module

    # Use setattr to reliably patch the BASE_DIR variable in the module.
    monkeypatch.setattr(server_module, "BASE_DIR", str(test_directory))
    yield


@pytest.fixture
def client(patched_app_state) -> TestClient:
    """Provides a FastAPI TestClient.

    It depends on `patched_app_state` to ensure the application is
    correctly configured before the client is created.
    """
    with TestClient(app) as test_client:
        yield test_client

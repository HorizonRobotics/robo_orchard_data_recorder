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

import pytest
from fastapi.testclient import TestClient
from robo_orchard_file_server.simple_file_server import (
    generate_directory_listing,
    guess_content_type,
    parse_byte_range,
)


class TestHelperFunctions:
    """Unit tests for standalone helper functions."""

    @pytest.mark.parametrize(
        "range_header, expected",
        [
            ("bytes=0-100", (0, 100)),
            ("bytes=50-", (50, None)),
            ("bytes=0-0", (0, 0)),
            (" bytes=10-20 ", (10, 20)),
        ],
    )
    def test_parse_byte_range_valid(self, range_header, expected):
        """Tests valid byte range parsing."""
        assert parse_byte_range(range_header) == expected

    @pytest.mark.parametrize(
        "invalid_range", ["bytes=100-50", "bytes=abc", "invalid"]
    )
    def test_parse_byte_range_invalid(self, invalid_range):
        """Tests that invalid byte ranges raise ValueError."""
        with pytest.raises(ValueError):
            parse_byte_range(invalid_range)

    def test_parse_byte_range_empty_string(self):
        """Tests that an empty string is parsed as no range."""
        assert parse_byte_range("") == (None, None)
        assert parse_byte_range("   ") == (None, None)

    @pytest.mark.parametrize(
        "filepath, expected_type",
        [
            ("file.html", "text/html"),
            ("image.jpg", "image/jpeg"),
            ("script.js", "application/javascript"),
            ("data.mcap", "application/octet-stream"),
            ("unknown.ext", "application/octet-stream"),
        ],
    )
    def test_guess_content_type(self, filepath, expected_type):
        """Tests MIME type guessing."""
        assert guess_content_type(filepath) == expected_type

    def test_generate_directory_listing(self, patched_app_state):
        """Tests HTML directory listing generation."""
        listing_html = generate_directory_listing(
            path="", request_url="http://testserver/"
        )
        assert listing_html is not None
        assert "<h1>Directory listing for /</h1>" in listing_html
        assert 'href="/test.txt"' in listing_html
        assert 'href="/subdir"' in listing_html


class TestApiEndpoints:
    """Integration tests for the FastAPI endpoints."""

    def test_get_root_directory(self, client: TestClient):
        """Tests serving the root directory listing."""
        response = client.get("/")
        assert response.status_code == 200
        assert "text/html" in response.headers["content-type"]
        content = response.text
        assert 'href="/subdir"' in content
        assert 'href="/test.txt"' in content

    def test_get_file(self, client: TestClient):
        """Tests serving a complete file."""
        response = client.get("/test.txt")
        assert response.status_code == 200
        assert response.text == "This is a test file."
        assert response.headers["content-type"] == "application/octet-stream"
        assert response.headers["content-length"] == str(
            len("This is a test file.")
        )

    def test_get_nested_file(self, client: TestClient):
        """Tests serving a file from a subdirectory."""
        response = client.get("/subdir/nested.txt")
        assert response.status_code == 200
        assert response.text == "Nested content."

    def test_file_not_found(self, client: TestClient):
        """Tests requesting a non-existent file."""
        response = client.get("/nonexistent.file")
        assert response.status_code == 404
        assert response.text == "File not found"

    def test_head_request(self, client: TestClient):
        """Tests a HEAD request for file metadata."""
        response = client.head("/test.txt")
        assert response.status_code == 200
        assert response.text == ""  # HEAD response has no body
        assert "last-modified" in response.headers
        assert response.headers["content-length"] == str(
            len("This is a test file.")
        )

    def test_partial_content_range_request(self, client: TestClient):
        """Tests a valid byte range request."""
        headers = {"Range": "bytes=0-3"}
        response = client.get("/test.txt", headers=headers)
        assert response.status_code == 206  # Partial Content
        assert response.text == "This"
        assert response.headers["content-length"] == "4"
        assert (
            response.headers["content-range"]
            == f"bytes 0-3/{len('This is a test file.')}"
        )

    def test_invalid_range_request(self, client: TestClient):
        """Tests a request with an unsatisfiable range."""
        # File size is 20
        headers = {"Range": "bytes=100-200"}
        response = client.get("/test.txt", headers=headers)
        assert response.status_code == 416
        assert response.text == "Requested Range Not Satisfiable"

    def test_options_request(self, client: TestClient):
        """Tests the OPTIONS request for CORS preflight."""
        response = client.options("/test.txt")
        assert response.status_code == 200
        assert "access-control-allow-origin" in response.headers
        assert response.headers["access-control-allow-origin"] == "*"

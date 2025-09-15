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

import argparse
import email.utils
import os
import re
from datetime import datetime
from html import escape  # Used for HTML escaping to prevent XSS
from typing import Optional

import aiofiles
from fastapi import FastAPI, HTTPException, Request, Response
from fastapi.responses import FileResponse, HTMLResponse, StreamingResponse

from robo_orchard_file_server.utils import find_free_port

app = FastAPI()

BYTE_RANGE_RE = re.compile(r"bytes=(\d+)-(\d+)?$")

CORS_HEADERS = {
    "Access-Control-Allow-Origin": "*",
    "Access-Control-Allow-Methods": "GET, HEAD, OPTIONS",
    "Access-Control-Allow-Headers": "*",
    "Access-Control-Expose-Headers": "Accept-Ranges, Content-Range",
}

BASE_DIR = os.getenv("ROBO_ORCHARD_SIMPLE_FILE_SERVER_BASE_DIR", os.getcwd())
if not os.path.isdir(BASE_DIR):
    raise FileNotFoundError(f"Directory {BASE_DIR} does not exist")
print(f"Serving files from: {BASE_DIR}")


def parse_byte_range(byte_range: str) -> tuple[Optional[int], Optional[int]]:
    """Parse the byte range from the Range header.

    Args:
        byte_range (str): The Range header value (e.g., "bytes=0-1023").

    Returns:
        tuple: A tuple of (start, stop) byte positions, where
            either can be None.

    Raises:
        ValueError: If the byte range is invalid.
    """
    # First, strip whitespace from the input string.
    stripped_range = byte_range.strip()
    if not stripped_range:
        return None, None

    # Perform the regex match on the stripped string.
    m = BYTE_RANGE_RE.match(stripped_range)
    if not m:
        raise ValueError("Invalid byte range")

    first, last = [int(x) if x else None for x in m.groups()]
    if last is not None and last < first:
        raise ValueError("Invalid byte range")
    return first, last


async def copy_byte_range(
    infile, start: int, stop: int, chunk_size: int = 16 * 1024
):
    """Asynchronously read a specific byte range from a file.

    Args:
        infile (str): Path to the file.
        start (int): Starting byte position.
        stop (int): Ending byte position.
        chunk_size (int): Size of chunks to read at a time (default: 16KB).

    Yields:
        bytes: Chunks of the file content within the specified range.
    """
    async with aiofiles.open(infile, "rb") as f:
        await f.seek(start)
        remaining = stop - start + 1
        while remaining > 0:
            to_read = min(chunk_size, remaining)
            chunk = await f.read(to_read)
            if not chunk:
                break
            remaining -= len(chunk)
            yield chunk


def guess_content_type(filepath: str) -> str:
    """Guess the MIME type of a file based on its extension.

    Args:
        filepath (str): Path to the file.

    Returns:
        str: The guessed MIME type (defaults to "application/octet-stream").
    """
    ext = os.path.splitext(filepath)[1].lower()
    mime_types = {
        ".html": "text/html",
        ".css": "text/css",
        ".js": "application/javascript",
        ".png": "image/png",
        ".jpg": "image/jpeg",
        ".gif": "image/gif",
        ".mcap": "application/octet-stream",
    }
    return mime_types.get(ext, "application/octet-stream")


def get_base_headers(file_stat=None) -> dict:
    """Generate base HTTP headers for responses.

    Args:
        file_stat (os.stat_result, optional): File stat object for
            additional metadata.

    Returns:
        dict: A dictionary of HTTP headers.
    """
    headers = {"Accept-Ranges": "bytes", **CORS_HEADERS}
    if file_stat:
        last_modified = datetime.fromtimestamp(file_stat.st_mtime)
        headers["Last-Modified"] = email.utils.format_datetime(last_modified)
    return headers


def generate_directory_listing(path: str, request_url: str) -> str | None:
    """Generate an HTML directory listing for the given path.

    Args:
        path (str): The relative path within the base directory.
        request_url (str): The full request URL.

    Returns:
        str: HTML content for the directory listing, or None if the path is
        not a directory.
    """
    full_path = os.path.join(BASE_DIR, path)
    if not os.path.isdir(full_path):
        return None

    items = os.listdir(full_path)
    items.sort()  # Sort alphabetically

    html = [
        "<!DOCTYPE html>",
        "<html>",
        "<head><title>Directory listing for /{}</title></head>".format(
            escape(path)
        ),
        "<body>",
        "<h1>Directory listing for /{}</h1>".format(escape(path)),
        "<hr>",
        "<ul>",
    ]

    # Add parent directory link (if not root)
    if path:
        parent_path = os.path.dirname(path.rstrip("/"))
        html.append(f'<li><a href="/{parent_path}">../</a></li>')

    # Add file and directory list
    for item in items:
        item_path = os.path.join(path, item).lstrip("/")
        full_item_path = os.path.join(full_path, item)
        if os.path.isdir(full_item_path):
            item_display = f"{item}/"
        else:
            item_display = item
        html.append(
            f'<li><a href="/{escape(item_path)}">{escape(item_display)}</a></li>'  # noqa: E501
        )

    html.extend(
        [
            "</ul>",
            "<hr>",
            "</body>",
            "</html>",
        ]
    )
    return "\n".join(html)


@app.exception_handler(HTTPException)
async def http_exception_handler(request: Request, exc: HTTPException):
    """Handle HTTP exceptions and return a response with CORS headers.

    Args:
        request (Request): The incoming request.
        exc (HTTPException): The exception to handle.

    Returns:
        Response: A response with the exception details and CORS headers.
    """
    return Response(
        content=exc.detail, status_code=exc.status_code, headers=CORS_HEADERS
    )


@app.options("/{filepath:path}")
async def options_handler(filepath: str):
    """Handle OPTIONS requests for CORS preflight.

    Args:
        filepath (str): The requested file path.

    Returns:
        Response: A 200 response with CORS headers.
    """
    return Response(status_code=200, headers=CORS_HEADERS)


@app.get("/{filepath:path}")
async def serve_file(request: Request, filepath: str):
    """Serve a file or directory listing.

    Args:
        request (Request): The incoming request.
        filepath (str): The requested file path.

    Returns:
        Response: A file response, directory listing, or streaming response
        for partial content.

    Raises:
        HTTPException: If the file or directory is not found, or if the
        range is invalid.
    """
    full_path = os.path.join(BASE_DIR, filepath)

    # If it's a directory, return a file listing
    if os.path.isdir(full_path):
        html_content = generate_directory_listing(filepath, str(request.url))
        if html_content:
            return HTMLResponse(content=html_content, status_code=200)
        raise HTTPException(status_code=404, detail="Directory not found")

    # If it's a file, serve it
    if not os.path.isfile(full_path):
        raise HTTPException(status_code=404, detail="File not found")

    file_stat = os.stat(full_path)
    file_size = file_stat.st_size
    headers = get_base_headers(file_stat)

    range_header = request.headers.get("Range")
    if not range_header:
        return FileResponse(
            full_path,
            headers=headers,
            media_type=guess_content_type(full_path),
        )

    try:
        start, stop = parse_byte_range(range_header)
    except ValueError:
        raise HTTPException(status_code=400, detail="Invalid byte range")

    if start is None:
        start = 0
    if stop is None or stop >= file_size:
        stop = file_size - 1

    if start >= file_size:
        raise HTTPException(
            status_code=416, detail="Requested Range Not Satisfiable"
        )

    content_length = stop - start + 1
    headers.update(
        {
            "Content-Range": f"bytes {start}-{stop}/{file_size}",
            "Content-Length": str(content_length),
        }
    )

    async def stream_response():
        async for chunk in copy_byte_range(full_path, start, stop):
            yield chunk

    return StreamingResponse(
        stream_response(),
        status_code=206,
        headers=headers,
        media_type=guess_content_type(full_path),
    )


@app.head("/{filepath:path}")
async def head_file(request: Request, filepath: str):
    """Handle HEAD requests to return file metadata.

    Args:
        request (Request): The incoming request.
        filepath (str): The requested file path.

    Returns:
        Response: A response with file metadata headers.

    Raises:
        HTTPException: If the file or directory is not found.
    """
    full_path = os.path.join(BASE_DIR, filepath)

    if not os.path.exists(full_path) or os.path.isdir(full_path):
        raise HTTPException(status_code=404, detail="Not found")

    file_stat = os.stat(full_path)
    headers = get_base_headers(file_stat)

    # Add the Content-Length header based on the file size
    headers["Content-Length"] = str(file_stat.st_size)

    return Response(
        status_code=200,
        headers=headers,
        media_type=guess_content_type(full_path),
    )


def main():
    parser = argparse.ArgumentParser(
        description="Robo Orchard Simple File Server"
    )
    parser.add_argument(
        "--port",
        type=int,
        default=None,
        help="Port to run the server on (default: random available port)",
    )
    args = parser.parse_args()

    port = args.port if args.port is not None else find_free_port()
    print(f"Server will run on port: {port}")

    import uvicorn

    uvicorn.run(app, host="0.0.0.0", port=port)


if __name__ == "__main__":
    main()

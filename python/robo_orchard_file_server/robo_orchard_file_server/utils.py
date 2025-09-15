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

import socket

__all__ = ["find_free_port"]


def find_free_port() -> int:
    """Find an available free port on the local machine.

    This function creates a temporary socket, binds it to port 0
    (which lets the OS assign a free port), and then retrieves and
    returns that port number. The socket is automatically
    closed after use.

    Returns:
        int: The free port number assigned by the operating system.

    Examples:
        >>> port = find_free_port()
        >>> print(port)  # Outputs a free port number, e.g., 49152
    """
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind(("", 0))
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        return s.getsockname()[1]

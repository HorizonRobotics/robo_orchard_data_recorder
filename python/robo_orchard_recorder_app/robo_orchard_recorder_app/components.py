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

import functools
import json
import os
import subprocess
from abc import ABCMeta, abstractmethod
from typing import Literal

import polling2
import pydantic
import streamlit as st
from streamlit.components.v1 import iframe

from robo_orchard_recorder_app.config import FoxgloveCfg, LaunchCfg, TaskCfg
from robo_orchard_recorder_app.utils import (
    check_process,
    remove_path,
    start_process,
    stop_process,
    time_str_now,
)


class EpisodeCounter:
    """A simple counter class for tracking episode numbers.

    Attributes:
        idx (int): The current episode count.
    """

    def __init__(self):
        self.idx: int = 0

    def current(self) -> int:
        """Returns the current episode count.

        Returns:
            int: Current episode number
        """
        return self.idx

    def add(self) -> int:
        """Increments the episode count by 1.

        Returns:
            int: New episode number
        """
        self.idx += 1
        return self.idx

    def sub(self) -> int:
        """Decrements the episode count by 1.

        Returns:
            int: New episode number
        """
        self.idx -= 1
        return self.idx


class CollectingState(pydantic.BaseModel):
    """Manages the state of data collection process."""

    model_config = pydantic.ConfigDict(arbitrary_types_allowed=True)

    session_time_str: str = pydantic.Field(default_factory=time_str_now)
    """Timestamp of the current session."""

    user_name: str = ""
    """Name of the user collecting data."""

    task_name: str = ""
    """Name of the current task."""

    data_root: str = ""
    """Root directory for data storage."""

    log_root: str = ""
    """Root directory for log storage."""

    current_data_uri: str = ""
    """Current data storage URI."""

    current_log_uri: str = ""
    """Current log storage URI."""

    is_configured: bool = False
    """Configuration status."""

    episode_counters: dict[str, EpisodeCounter] = pydantic.Field(
        default_factory=lambda: dict()
    )
    """Dictionary of episode counters per user-task pair."""

    @property
    def episode_counter(self) -> EpisodeCounter:
        """Gets or creates an episode counter for the current user-task pair.

        Returns:
            EpisodeCounter: Counter instance for the current user and task
        """
        key = "{}:{}".format(self.user_name, self.task_name)
        if key not in self.episode_counters:
            self.episode_counters[key] = EpisodeCounter()
        return self.episode_counters[key]


class ComponentBase(metaclass=ABCMeta):
    """Abstract base class for UI components."""

    @abstractmethod
    def __call__(self, *args, **kwargs):
        """Abstract method to execute the component."""
        pass


class TaskConfigComponent(ComponentBase):
    """Component for managing task configuration."""

    def __init__(self, key: str = "task_cfg"):
        """Constructor.

        Args:
            key (str): Session state key for storing configuration

        Raises:
            KeyError: If the specified key is not found in session state
        """
        if key not in st.session_state:
            raise KeyError(f"Cannot find {key} in session_state")
        self.session_key = key
        self.edit_panel_key = f"{key}_TaskConfigEditPanel"

    @property
    def cfg(self) -> TaskCfg:  # type: ignore
        return st.session_state[self.session_key]

    @cfg.setter
    def cfg(self, value: TaskCfg):  # type: ignore
        st.session_state[self.session_key] = value

    @property
    def json_cfg(self) -> str:
        return self.cfg.model_dump_json(indent=4)

    def _edit_component(self):
        def _callback():
            try:
                self.cfg = TaskCfg.model_validate_json(
                    st.session_state[self.edit_panel_key]
                )
            except pydantic.ValidationError as e:
                st.error(f"❌ JSON schema error: {e}")

        st.text_area(
            "Edit Task Config",
            value=self.json_cfg,
            height=256,
            key=self.edit_panel_key,
            on_change=_callback,
        )

    def _import_component(self):
        uploaded_file = st.file_uploader(
            "Load task config", type=["json"], help="Only support json config"
        )
        if uploaded_file:
            try:
                self.cfg = TaskCfg.model_validate_json(
                    uploaded_file.getvalue().decode()
                )
                st.toast("✅ Custom config is loaded.", icon="✅")
            except pydantic.ValidationError as e:
                st.error(f"❌ Config is error: {e}")

    def _export_component(self):
        st.download_button(
            label="⬇️ Export task config",
            data=self.json_cfg,
            file_name="robo_orchard_recorder_app_task_config_{}.json".format(
                time_str_now()
            ),
            mime="application/json",
            use_container_width=True,
        )

    def __call__(self):
        """Renders the task configuration management UI."""
        with st.expander(":gear: Task Config Management", expanded=False):
            left_col, right_col = st.columns([0.8, 0.2])

            with left_col:
                self._edit_component()

            with right_col:
                self._import_component()
                self._export_component()


class ControlComponent(ComponentBase):
    """Component for controlling data collection process."""

    OTHERS_VALUE = "others"
    INVALID_VALUES = [OTHERS_VALUE, "", None]

    def __init__(
        self,
        data_record_config_file: str,
        app_cache_directory: str,
        launch_cfg_key: str = "launch_cfg",
        task_cfg_key: str = "task_cfg",
        collecting_state_key: str = "collecting_state",
    ):
        """Constuctor.

        Args:
            data_record_config_file (str): Path to data recording config file
            app_cache_directory (str): Directory for application cache
            task_cfg_key (str): Session state key for task config
            collecting_state_key (str): Session state key for collecting state
        """
        self.data_record_config_file = data_record_config_file
        self.app_cache_directory = app_cache_directory
        self._launch_cfg_key = launch_cfg_key
        self._task_cfg_key = task_cfg_key
        self._collecting_state_key = collecting_state_key
        self._temp_cfg = {
            "user_name": "",
            "task_name": "",
            "instruction": "",
        }
        self.cfg = {
            "user_name": "",
            "task_name": "",
            "instruction": "",
        }
        self._recorder_process: subprocess.Popen | None = None

    @property
    def launch_cfg(self) -> LaunchCfg:
        return st.session_state[self._launch_cfg_key]

    @property
    def task_cfg(self) -> TaskCfg:
        return st.session_state[self._task_cfg_key]

    @property
    def task_cfg_json(self) -> str:
        return self.task_cfg.model_dump_json(indent=4)

    @property
    def collecting_state(self) -> CollectingState:
        return st.session_state[self._collecting_state_key]

    def _update_text_callback(self, src_key: str, dst_key: str):
        value = st.session_state[src_key]
        self._temp_cfg[dst_key] = value
        if value not in self.INVALID_VALUES:
            self.cfg[dst_key] = value

    def _show_panel(self):
        with st.expander(":gear: Current config", expanded=True):
            st.json(self.cfg, expanded=True)

    def _input_panel(
        self, input_type: Literal["user_name", "task_name", "instruction"]
    ):
        def _get_options() -> list[str]:
            if input_type == "user_name":
                options = self.task_cfg.available_collectors
            elif input_type == "task_name":
                options = list(self.task_cfg.available_tasks.keys())
            elif input_type == "instruction":
                options = self.task_cfg.available_tasks.get(
                    self.cfg["task_name"], []
                )
            else:
                raise ValueError(
                    "Unsupported input type: {}".format(input_type)
                )
            return options

        select_col, input_col = st.columns(2)
        with select_col:
            options = _get_options()
            if not options:
                index = 0
                self._temp_cfg[input_type] = self.OTHERS_VALUE
            elif self.cfg[input_type] in options:
                index = options.index(self.cfg[input_type])
            else:
                index = None
            st.selectbox(
                f"Select {input_type}",
                options=options + [self.OTHERS_VALUE],
                index=index,
                format_func=lambda x: x,
                disabled=False,
                key=f"{input_type}_select_input",
                on_change=functools.partial(
                    self._update_text_callback,
                    src_key=f"{input_type}_select_input",
                    dst_key=input_type,
                ),
                label_visibility="visible",
            )
        with input_col:
            st.text_input(
                f"Input {input_type}",
                value="",
                on_change=functools.partial(
                    self._update_text_callback,
                    src_key=f"{input_type}_text_input",
                    dst_key=input_type,
                ),
                key=f"{input_type}_text_input",
                disabled=self._temp_cfg[input_type] != self.OTHERS_VALUE,
            )

    def _dump_task_cfg(self):
        save_directory = os.path.join(
            self.app_cache_directory,
            "task_configs",
            time_str_now(),
        )
        os.makedirs(save_directory, exist_ok=True)
        with open(os.path.join(save_directory, "app_task.json"), "w") as fw:
            fw.write(self.task_cfg_json)
        with open(
            os.path.join(
                self.app_cache_directory,
                "task_configs",
                "latest.json",
            ),
            "w",
        ) as fw:
            fw.write(self.task_cfg_json)

    def _confirm_panel(self):
        if st.button(":label: Confirm collecting config"):
            is_check_ok = True
            for key in ["user_name", "task_name"]:
                if self.cfg[key] in self.INVALID_VALUES:
                    st.error(
                        "Invalid value for {}: {}".format(key, self.cfg[key])
                    )
                    is_check_ok = False

            if not is_check_ok:
                self.collecting_state.is_configured = False
                return

            data_root = os.path.join(
                self.launch_cfg.workspace,  # type: ignore
                self.collecting_state.session_time_str,
                "data",
                self.cfg["user_name"],  # type: ignore
                self.cfg["task_name"],  # type: ignore
            )
            os.makedirs(data_root, exist_ok=True)

            log_root = os.path.join(
                self.launch_cfg.workspace,  # type: ignore
                self.collecting_state.session_time_str,
                "logs",
                self.cfg["user_name"],  # type: ignore
                self.cfg["task_name"],  # type: ignore
            )
            os.makedirs(log_root, exist_ok=True)

            self.collecting_state.user_name = self.cfg["user_name"]
            self.collecting_state.task_name = self.cfg["task_name"]
            self.collecting_state.data_root = data_root
            self.collecting_state.log_root = log_root
            self.collecting_state.is_configured = True

            # update global config
            if self.cfg["user_name"] not in self.task_cfg.available_collectors:
                self.task_cfg.available_collectors.append(
                    self.cfg["user_name"]
                )
            if self.cfg["task_name"] not in self.task_cfg.available_tasks:
                self.task_cfg.available_tasks[self.cfg["task_name"]] = []
            if self.cfg["instruction"] not in self.INVALID_VALUES:
                if (
                    self.cfg["instruction"]
                    not in self.task_cfg.available_tasks[self.cfg["task_name"]]
                ):
                    self.task_cfg.available_tasks[
                        self.cfg["task_name"]
                    ].append(self.cfg["instruction"])

            self._dump_task_cfg()

            st.rerun()

    def _record_panel(self):
        def _start_impl(log_msg_place_holder):
            if self._recorder_process is not None:
                log_msg_place_holder.markdown(
                    ":red[An episode is recorded, please decide to save or not first!]"  # noqa: E501
                )

            time_str = time_str_now()
            data_uri = os.path.join(
                self.collecting_state.data_root,
                f"episode_{time_str}",
            )
            log_uri = os.path.join(
                self.collecting_state.log_root,
                f"episode_{time_str}",
            )
            log_msg_place_holder.markdown(
                f"Start recording data to: :green[{data_uri}] and logs to: :green[{log_uri}]"  # noqa: E501
            )

            if self.data_record_config_file:
                command = f"""
                set -ex
                ros2 run robo_orchard_data_ros2 mcap_recorder \
                    --ros-args \
                    -p config_file:="{self.data_record_config_file}" \
                    -p uri:="{data_uri}"
                """
            else:
                command = f"""
                set -ex
                ros2 run robo_orchard_data_ros2 mcap_recorder \
                    --ros-args \
                    -p uri:="{data_uri}"
                """
            self._recorder_process = start_process(
                command, shell=True, executable="/bin/bash", min_live_time=0
            )
            self.collecting_state.current_data_uri = data_uri
            self.collecting_state.current_log_uri = log_uri

        def _check_is_recording():
            recording_flag = os.path.join(
                self.collecting_state.current_data_uri, "__RECORDING__"
            )
            return os.path.exists(recording_flag)

        def _stop_impl(log_msg_place_holder):
            if self._recorder_process is None:
                log_msg_place_holder.markdown(
                    ":red[Please start recording first]"
                )
                return

            stop_process(self._recorder_process)
            self.collecting_state.episode_counter.add()
            log_msg_place_holder.markdown(
                "Save {}-th episode to :green[{}]".format(
                    self.collecting_state.episode_counter.current(),
                    self.collecting_state.current_data_uri,
                )
            )
            # write data meta
            if os.path.exists(self.collecting_state.current_data_uri):
                with open(
                    os.path.join(
                        self.collecting_state.current_data_uri,
                        "episode_meta.json",
                    ),
                    "w",
                ) as fw:
                    json.dump(self.cfg, fw, indent=4)
            self._recorder_process = None
            log_msg_place_holder.markdown(":green[Recorder stopped]")

        if self.collecting_state.is_configured:
            log_msg_place_holder = st.empty()

            left_col, right_col = st.columns(2)

            with left_col:
                if st.button(
                    "Start",
                    on_click=_start_impl,
                    args=(log_msg_place_holder,),
                    disabled=self._recorder_process is not None,
                ):
                    if self._recorder_process is not None:
                        with st.spinner(
                            "Starting, please wait...", show_time=True
                        ):
                            try:
                                check_process(
                                    self._recorder_process, min_live_time=1.0
                                )
                                polling2.poll(
                                    _check_is_recording, timeout=10.0, step=0.1
                                )
                                st.rerun()
                            except subprocess.CalledProcessError as e:
                                self._recorder_process = None
                                log_msg_place_holder.error(
                                    "Failed to start recording process! Error msgs:\n{}\n".format(  # noqa: E501
                                        e.stderr.decode("utf-8")
                                    )
                                )
                            except polling2.TimeoutException:
                                log_msg_place_holder.error(
                                    "Failed to start recording process! Please check again."  # noqa: E501
                                )

            with right_col:
                st.button(
                    "Stop",
                    on_click=_stop_impl,
                    args=(log_msg_place_holder,),
                    disabled=self._recorder_process is None,
                )

    def __call__(self):
        """Renders the control panel UI."""
        self._show_panel()
        self._input_panel("user_name")
        self._input_panel("task_name")
        self._input_panel("instruction")
        self._confirm_panel()
        self._record_panel()


class SideBarComponent(ComponentBase):
    """Sidebar component for managing recorded files."""

    def __init__(
        self,
        foxglove_cfg: FoxgloveCfg,
        file_server_host: str,
        collecting_state_key: str = "collecting_state",
    ):
        """Constructor.

        Args:
            foxglove_cfg (FoxgloveCfg): Foxglove configuration
            file_server_host (str): File server host address
            collecting_state_key (str): Session state key for collecting state
        """
        self.foxglove_cfg: FoxgloveCfg = foxglove_cfg
        self.file_server_host: str = file_server_host
        self._collecting_state_key = collecting_state_key
        self._selected_uri: str | None = None
        self._delete_flags = dict()

    @property
    def collecting_state(self) -> CollectingState:
        return st.session_state[self._collecting_state_key]

    def _delete_callback(self, uri: str):
        if self._delete_flags.get(uri, False):
            st.markdown(f":red[Already tried deleting {uri}]")
        try:
            with st.spinner("Deleting"):
                remove_path(uri)
            self._delete_flags[uri] = True
            self.collecting_state.episode_counter.sub()
            self._selected_uri = None  # reset selected state
        except FileNotFoundError:
            st.error(f"Cannot found recording uri: {uri}")

    @st.fragment
    def _sidebar_panel(self):
        for idx, uri in enumerate(
            sorted(os.listdir(self.collecting_state.data_root))
        ):
            with st.container():
                if st.button(uri):
                    if self._selected_uri == uri:
                        self._selected_uri = None
                    else:
                        self._selected_uri = uri

                if self._selected_uri == uri:
                    data_uri = os.path.join(
                        self.collecting_state.data_root, uri
                    )

                    left_col, right_col = st.columns(2)
                    with left_col:
                        st.link_button(
                            "Visualize",
                            url=self.foxglove_cfg.get_remote_file_url(
                                remote_file="{}/{}".format(
                                    self.file_server_host,
                                    os.path.abspath(
                                        os.path.join(data_uri, f"{uri}_0.mcap")
                                    ),
                                )
                            ),
                        )
                    with right_col:
                        st.button(
                            "Delete",
                            key=f"delete_{idx}",
                            on_click=self._delete_callback,
                            args=(data_uri,),
                        )

    def __call__(self):
        """Renders the sidebar UI."""

        st.sidebar.title("📁 File")
        st.sidebar.markdown(f"User: {self.collecting_state.user_name}")
        st.sidebar.markdown(f"Task: {self.collecting_state.task_name}")

        if not self.collecting_state.is_configured or not os.path.exists(
            self.collecting_state.data_root
        ):
            return

        with st.sidebar:
            self._sidebar_panel()


class FoxgloveIFrameComponent(ComponentBase):
    """Component for displaying Foxglove."""

    def __init__(self, cfg: FoxgloveCfg):
        """Constructor.

        Args:
            cfg (FoxgloveCfg): Foxglove configuration
        """
        self.cfg = cfg

    def _render_link_button(self):
        css_html = """
        <style>
            /* Target the container for the columns */
            div[data-testid="stHorizontalBlock"] {
                /* Make the columns themselves align to the center vertically */
                align-items: center;
            }

            /* Optional: if you still want some space on top */
            .st-emotion-cache-z5fcl4 {
                padding-top: 2rem;
            }

            /* Style for the button, now without border and with a link-like look */
            .custom-button {
                border: none;
                padding: 0.5rem 1rem;
                border-radius: 0.25rem;
                background-color: transparent;
                color: #0068c9;
                text-align: center;
                text-decoration: none;
                font-size: 1rem;
                font-weight: 600;
                cursor: pointer;
                transition: background-color 0.3s;
            }
            .custom-button:hover {
                background-color: #f0f2f6;
            }
        </style>
        """  # noqa: E501
        st.markdown(css_html, unsafe_allow_html=True)

        button_html = f"""
        <div style="display: flex; justify-content: center;">
            <a href="{self.cfg.get_websocket_url()}" target="_blank" class="custom-button">
                🚀 Launch Visualization
            </a>
        </div>
        """  # noqa: E501
        st.markdown(button_html, unsafe_allow_html=True)

    def __call__(self):
        """Renders the Foxglove iframe."""
        with st.spinner("Loading Foxglove"):
            if self.cfg.display_type == "iframe":
                iframe(
                    src=self.cfg.get_websocket_url(),
                    width=self.cfg.width,
                    height=self.cfg.height,
                )
            elif self.cfg.display_type == "link_button":
                self._render_link_button()
            else:
                raise NotImplementedError(
                    f"Invalid display type: {self.cfg.display_type}"
                )

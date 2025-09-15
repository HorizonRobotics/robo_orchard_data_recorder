# Put it first so that "make" without argument is like "make help".
ROOTDIR = $(CURDIR)
version_type := local
EXTRA_ARGS =
PIP_ARGS =
BUILD_ARGS =
COMMIT_UNIXTIME := $(shell git log -n 1 --pretty='format:%ct')
COMMIT_DATETIME := $(shell date -d @${COMMIT_UNIXTIME} +'%Y%m%d%H%M%S')
COMMIT_ID := $(shell git rev-parse --short HEAD)

install:
	@make -C python/robo_orchard_file_server install EXTRA_ARGS="${EXTRA_ARGS}" BUILD_ARGS="${BUILD_ARGS}" PIP_ARGS="${PIP_ARGS}" version_type=${version_type}
	@make -C python/robo_orchard_recorder_app install EXTRA_ARGS="${EXTRA_ARGS}" BUILD_ARGS="${BUILD_ARGS}" PIP_ARGS="${PIP_ARGS}" version_type=${version_type}

install-editable:
	@make -C python/robo_orchard_file_server install-editable EXTRA_ARGS="${EXTRA_ARGS}" BUILD_ARGS="${BUILD_ARGS}" PIP_ARGS="${PIP_ARGS}" version_type=${version_type}
	@make -C python/robo_orchard_recorder_app install-editable EXTRA_ARGS="${EXTRA_ARGS}" BUILD_ARGS="${BUILD_ARGS}" PIP_ARGS="${PIP_ARGS}" version_type=${version_type}

dist-build:
	@make -C python/robo_orchard_file_server dist-build EXTRA_ARGS="${EXTRA_ARGS}" BUILD_ARGS="${BUILD_ARGS}" PIP_ARGS="${PIP_ARGS}" version_type=${version_type}
	@make -C python/robo_orchard_recorder_app dist-build EXTRA_ARGS="${EXTRA_ARGS}" BUILD_ARGS="${BUILD_ARGS}" PIP_ARGS="${PIP_ARGS}" version_type=${version_type}

ros2_build:
	make -C ros2_package build_ros2

ros2_test:
	make -C ros2_package test

ros2_clean:
	make -C ros2_package clean

dev-env:
	@pip3 install -r scm/requirements.txt ${PIP_ARGS}
	@pre-commit install

auto-format:
	@python3 scm/lint/check_lint.py --auto_format

check-lint:
	@python3 scm/lint/check_lint.py
	@pre-commit run check-merge-conflict
	@pre-commit run check-license-header --all-files

doc:
	make -C docs html

doc-clean:
	make -C docs clean

test:
	make -C tests

test_ut:
	make -C tests test_ut

test_it:
	make -C tests test_it

show-args:
	@echo "PIP_ARGS: $(PIP_ARGS)"
	@echo "BUILD_ARGS: $(BUILD_ARGS)"
	@echo "EXTRA_ARGS: $(EXTRA_ARGS)"
# Contribution Guide

## Install development requirements

```bash
make dev-env
```

## Install by editable mode

```bash
make install-editable
make ros2_clean && make ros2_build
```

## Lint

```bash
make check-lint
```

## Auto format

```bash
make auto-format
```

## Build docs

```bash
make doc
```

## Preview docs

```bash
cd build/docs_build/html
python3 -m http.server {PORT}
# open browser: http://localhost:{PORT}
```

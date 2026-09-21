# Container configuration

Run commands from the repository root. The wrapper supplies the project directory
so Compose keeps the same project name, root `.env` lookup, calibration mount,
and CSV output paths.

```bash
./fr3-stack build
./fr3-stack up cart
./fr3-stack down
```

For direct Compose access:

```bash
docker compose --project-directory . -f containers/compose.yml config
docker compose --project-directory . -f containers/compose.yml build
```

For a direct image build:

```bash
docker build -f containers/Dockerfile -t fr3-stack:latest .
```

The build context is the repository root. Starting services operates robot or
sensor hardware; configuration inspection and image builds do not start them.

FROM python:3.10-slim-bookworm AS builder

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake ninja-build \
    && rm -rf /var/lib/apt/lists/*
WORKDIR /src
COPY pyproject.toml README.md CMakeLists.txt ./
COPY cpp ./cpp
COPY src ./src
RUN pip wheel --no-cache-dir --wheel-dir /wheels .

FROM python:3.10-slim-bookworm AS runtime

ENV PYTHONUNBUFFERED=1 \
    RMP_DATA_DIR=/data \
    RMP_IMPORT_ROOTS=/imports \
    RMP_HOST=0.0.0.0 \
    RMP_PORT=28200
COPY --from=builder /wheels /wheels
RUN pip install --no-cache-dir /wheels/*.whl && rm -rf /wheels
RUN useradd --create-home --uid 10001 rmp && mkdir -p /data /imports && chown -R rmp:rmp /data /imports
USER rmp
EXPOSE 28200
VOLUME ["/data", "/imports"]
HEALTHCHECK --interval=15s --timeout=3s --start-period=10s --retries=5 \
  CMD python -c "import urllib.request; urllib.request.urlopen('http://127.0.0.1:28200/healthz', timeout=2)" || exit 1
CMD ["robot-map-planner", "serve"]

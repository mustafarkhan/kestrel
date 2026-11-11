# Kestrel fusion node (and tools) as a container. The airborne node normally runs on the
# aircraft's companion computer from a virtual environment instead (see docs/deployment.md).
#
#   docker build -t kestrel .
#   docker run --rm -p 5020:5020 -v $PWD/site:/site:ro -v $PWD/out:/out kestrel \
#     fusion --site /site/site.yaml --listen 0.0.0.0:5020 --out /out/rap.jsonl \
#     --tls-ca /site/pki/ca.crt --tls-cert /site/pki/fusion.crt --tls-key /site/pki/fusion.key
FROM python:3.12-slim AS build
WORKDIR /src
COPY pyproject.toml README.md LICENSE ./
COPY src ./src
RUN pip install --no-cache-dir build && python -m build --wheel --outdir /dist

FROM python:3.12-slim
COPY --from=build /dist/*.whl /tmp/
RUN pip install --no-cache-dir /tmp/*.whl && rm /tmp/*.whl \
    && useradd --system --uid 10001 --home /nonexistent --shell /usr/sbin/nologin kestrel
USER 10001
EXPOSE 5020
ENTRYPOINT ["kestrel"]
CMD ["--help"]

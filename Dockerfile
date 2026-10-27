# Build, test and run the scenario suite in a clean Linux environment:
#
#   docker build -t haul-vcu .
#   docker run --rm haul-vcu                          # all scenarios, PASS/FAIL per check
#   docker run --rm haul-vcu haul-bench
#   docker run --rm -v "$PWD/out:/out" haul-vcu haul-sim --out /out scenarios/14_haul_cycle.json

FROM ubuntu:24.04 AS build
RUN apt-get update -q && apt-get install -y -q --no-install-recommends \
      g++ cmake ninja-build git ca-certificates \
    && rm -rf /var/lib/apt/lists/*
WORKDIR /src
COPY . .
RUN cmake --preset release && cmake --build --preset release && ctest --preset release

FROM ubuntu:24.04
RUN apt-get update -q && apt-get install -y -q --no-install-recommends libstdc++6 \
    && rm -rf /var/lib/apt/lists/*
# The tools look for the DBC under the source directory they were built from (/src), so the
# runtime image keeps the same layout.
WORKDIR /src
COPY --from=build /src/build/release/haul-sim /src/build/release/haul-vcu /src/build/release/haul-truck \
     /src/build/release/haul-can /src/build/release/haul-bench /usr/local/bin/
COPY dbc/ dbc/
COPY scenarios/ scenarios/
CMD ["sh", "-c", "haul-sim scenarios/*.json"]

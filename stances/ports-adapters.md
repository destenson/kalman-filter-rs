**Ports and adapters (hexagonal).** The core defines small interfaces (ports) for
every external or variable concern: backends, storage, I/O, clock, randomness,
logging/metrics. Adapters implement them at the edge. The core depends on nothing
concrete. Swapping an implementation never touches the core.
Avoid: core code importing concrete backends or telemetry libraries directly.
Likely failure mode: interfaces with one implementation; indirection a newcomer must trace.

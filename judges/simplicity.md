**Simplicity and clarity.** How much must a competent newcomer hold in their head to
make a correct change? Count concepts, not lines: types, traits/interfaces, indirection
hops from public API to the real work, feature flags, generic parameters. Reward code
whose structure mirrors the problem, with domain names and one obvious place for each
responsibility. Penalize dead code, duplication with drift, speculative abstraction,
and comments that explain *what* because the code can't.

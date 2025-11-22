# Contributing to Kalman Filters

Thank you for your interest in contributing to the Kalman Filters library! This document provides guidelines and information for contributors.

## Table of Contents

- [Getting Started](#getting-started)
- [Development Workflow](#development-workflow)
- [Running Tests](#running-tests)
- [Benchmarks and Performance](#benchmarks-and-performance)
- [Code Style and Standards](#code-style-and-standards)
- [Pull Request Process](#pull-request-process)
- [Progressive Refinement Proposals (PRPs)](#progressive-refinement-proposals-prps)

## Getting Started

### Prerequisites

- Rust 1.70 or later
- Cargo
- Git

### Setting Up Your Development Environment

1. Fork the repository on GitHub
2. Clone your fork:

   ```bash
   git clone https://github.com/YOUR_USERNAME/kalman-filters.git
   cd kalman-filters
   ```

3. Add the upstream repository:

   ```bash
   git remote add upstream https://github.com/destenson/kalman-filter-rs.git
   ```

4. Install development dependencies:

   ```bash
   cargo build
   cargo test
   ```

## Development Workflow

1. Create a new branch for your feature or fix:

   ```bash
   git checkout -b feature/your-feature-name
   ```

2. Make your changes and commit them with clear, descriptive commit messages:

   ```bash
   git commit -m "Add feature: brief description"
   ```

3. Keep your branch up to date with upstream:

   ```bash
   git fetch upstream
   git rebase upstream/master
   ```

4. Push your changes to your fork:

   ```bash
   git push origin feature/your-feature-name
   ```

5. Open a Pull Request on GitHub

## Running Tests

### Unit Tests

Run all unit tests:

```bash
cargo test
```

Run tests for a specific module:

```bash
cargo test --lib filter
```

Run tests with output:

```bash
cargo test -- --nocapture
```

### Integration Tests

Run integration tests:

```bash
cargo test --test '*'
```

### Test Coverage

We aim for 90%+ test coverage. You can check coverage locally using:

```bash
cargo tarpaulin --out Html
```

## Benchmarks and Performance

### Running Benchmarks Locally

To run the complete benchmark suite:

```bash
cargo bench --bench kalman_benchmarks
```

To run specific benchmarks:

```bash
cargo bench --bench kalman_benchmarks -- kalman_filter
```

### Benchmark Results

Criterion generates detailed HTML reports in `target/criterion/`. Open `target/criterion/report/index.html` in your browser to view interactive performance reports.

### Continuous Integration Benchmarks

Our CI system automatically runs benchmarks on every pull request and push to master. Here's how it works:

#### Baseline Storage

- **Master Branch**: Benchmarks run on every push to master and results are stored as baselines
- **Pull Requests**: Benchmarks are compared against the latest master baseline
- **Historical Tracking**: Performance data is tracked over time using GitHub Actions artifacts

#### Regression Detection

The CI will flag performance regressions based on these thresholds:

- **10% regression**: Warning issued, but build passes
- **20% regression**: Build fails, requires investigation

Example warning:

```text
::warning::Performance regression detected (>10%)
```

#### Benchmark CI Jobs

Our benchmark workflow includes two jobs:

1. **benchmark**: Runs benchmarks and stores results
   - Caches dependencies for faster runs
   - Stores criterion baselines between runs
   - Uploads HTML reports as artifacts
   - Comments on PRs with performance alerts

2. **benchmark-comparison**: Detailed PR comparisons
   - Checks out both base and PR branches
   - Runs benchmarks on both
   - Generates side-by-side comparison
   - Comments results on PR

#### Viewing Benchmark Results

1. **GitHub Actions**: Check the "Benchmarks" workflow in your PR
2. **Artifacts**: Download criterion HTML reports from the workflow artifacts
3. **PR Comments**: Automated comments show performance impact
4. **Logs**: Detailed benchmark output in the workflow logs

#### What to Do If Benchmarks Fail

If your PR triggers a performance regression:

1. **Review the criterion reports**: Download the artifacts to see which benchmarks regressed
2. **Investigate the cause**: Profile your code changes
3. **Optimize if necessary**: Fix the performance issue
4. **Document if intentional**: If the regression is expected (e.g., trading performance for correctness), document it in the PR description
5. **Re-run benchmarks**: Push your fixes and wait for CI to re-run

#### Benchmark Best Practices

- **Consistent environment**: CI uses the same runner configuration for all benchmarks
- **Cache warming**: Benchmarks run with criterion's default warm-up iterations
- **Statistical rigor**: Results are averaged over multiple iterations
- **Baseline comparison**: Always compared against master, not previous PR commits

#### Running Benchmarks Exactly Like CI

To reproduce CI benchmark results locally:

```bash
# Run with criterion's bencher output format
cargo bench --bench kalman_benchmarks -- --output-format bencher | tee benchmark_output.txt

# Compare against a baseline
cargo bench --bench kalman_benchmarks -- --save-baseline my-baseline
# ... make changes ...
cargo bench --bench kalman_benchmarks -- --baseline my-baseline
```

#### Benchmark Artifacts Retention

- Criterion HTML reports: 30 days
- Baseline data: Stored in GitHub Actions cache indefinitely (until evicted by cache limits)

### Performance Expectations

When contributing code:

- **No regressions**: Performance should not regress without good reason
- **Document trade-offs**: If you sacrifice performance for correctness or features, explain why
- **Benchmark new features**: Add benchmarks for new filter types or major features
- **Profile before optimizing**: Use profiling tools to identify actual bottlenecks

## Code Style and Standards

### Formatting

We use `rustfmt` for code formatting. Format your code before committing:

```bash
cargo fmt
```

### Linting

Run `clippy` to catch common mistakes:

```bash
cargo clippy -- -D warnings
```

### Documentation

- Add doc comments for all public APIs
- Include examples in doc comments where helpful
- Update relevant documentation files (README.md, docs/*, etc.)

### Code Quality

- Follow Rust idioms and best practices
- Prefer clarity over cleverness
- Add tests for new functionality
- Maintain or improve test coverage
- Avoid unnecessary dependencies

## Pull Request Process

1. **Ensure all tests pass**: `cargo test`
2. **Run clippy**: `cargo clippy -- -D warnings`
3. **Format code**: `cargo fmt`
4. **Run benchmarks locally**: `cargo bench` (if performance-related)
5. **Update documentation**: If you changed public APIs
6. **Write clear PR description**: Explain what, why, and how
7. **Link related issues**: Reference issue numbers if applicable
8. **Wait for CI**: All CI checks must pass (tests, benchmarks, clippy)
9. **Respond to reviews**: Address reviewer feedback
10. **Squash commits if requested**: Keep history clean

### PR Title Guidelines

Use conventional commit style:

- `feat: add new feature`
- `fix: resolve bug in filter`
- `docs: update README`
- `perf: optimize matrix multiplication`
- `test: add tests for UKF`
- `refactor: simplify code structure`
- `chore: update dependencies`

### What Happens After You Submit

1. **Automated checks run**: Tests, clippy, benchmarks
2. **Benchmarks are compared**: CI comments with performance impact
3. **Code review**: Maintainers review your code
4. **Discussion**: Address questions and feedback
5. **Approval**: Once approved and CI passes, your PR will be merged

## Progressive Refinement Proposals (PRPs)

For larger features or refactoring work, we use Progressive Refinement Proposals (PRPs).

### What is a PRP?

A PRP is a structured proposal for a 2-4 hour task that includes:

- Clear objectives
- Implementation steps
- Validation gates
- Success criteria

### PRP Directory

See `PRPs/README.md` for:

- Complete list of PRPs
- Implementation status
- Dependencies between PRPs
- How to create new PRPs

### Working on PRPs

1. Browse available PRPs in `PRPs/`
2. Choose one that matches your interests
3. Check dependencies are completed
4. Follow the implementation tasks
5. Run validation gates
6. Submit PR with PRP reference

### Creating a New PRP

1. Use the template in `PRPs/templates/prp_base.md`
2. Ensure scope is 2-4 hours
3. Include specific file locations and validation commands
4. Submit as PR with `[PRP]` prefix

## Getting Help

- **Questions**: Open a GitHub Discussion
- **Bugs**: Open a GitHub Issue with "bug" label
- **Feature requests**: Open a GitHub Issue with "enhancement" label
- **Security issues**: Email the maintainers privately (see README.md)

## Code of Conduct

Be respectful, inclusive, and professional. We're all here to build great software together.

## License

By contributing, you agree that your contributions will be licensed under the MIT License.

## Recognition

Contributors are recognized in:

- Git commit history
- Release notes for significant contributions
- AUTHORS file (if we create one)

Thank you for contributing to Kalman Filters!

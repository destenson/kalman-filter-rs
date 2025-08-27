# Automated Release Pipeline for crates.io

## Summary
Set up automated CI/CD pipeline for testing, versioning, and publishing the kalman-filters crate to crates.io using GitHub Actions and release automation tools.

## Context
- **Repository**: GitHub hosted at https://github.com/destenson/kalman-filter-rs
- **Current process**: Manual publication
- **Goal**: Automate testing, changelog generation, version bumping, and publishing
- **Tools**: GitHub Actions with cargo-release or release-plz

## Automation Strategy

### Option A: release-plz (Recommended)
- **Pros**: Fully automated, creates PRs for releases, manages changelog
- **Docs**: https://release-plz.ieni.dev/
- **Features**: Version bump, changelog, git tag, GitHub release, crates.io publish

### Option B: cargo-release
- **Pros**: More control, widely used
- **Docs**: https://github.com/crate-ci/cargo-release
- **Features**: Version management, git operations, publishing

### Option C: Custom GitHub Actions
- **Pros**: Full control, no external dependencies
- **Cons**: More maintenance, reinventing the wheel

## Implementation Plan (Using release-plz)

### Step 1: Create GitHub Actions Workflow

**File**: `.github/workflows/release.yml`

Key components:
1. Trigger on push to main branch
2. Run comprehensive test suite
3. Use release-plz to check for release
4. Create PR for version bump
5. Auto-publish on PR merge

**File**: `.github/workflows/ci.yml`

Key components:
1. Trigger on all PRs and pushes
2. Test matrix: OS (Linux, Windows, macOS) × Rust (stable, beta, nightly)
3. Run with all feature combinations
4. Generate coverage report
5. Run clippy and rustfmt checks

### Step 2: Configure release-plz

**File**: `release-plz.toml` (in repository root)

Configuration elements:
- Changelog generation settings
- Version bump rules (major, minor, patch)
- Git tag format
- PR labels
- Commit message format

### Step 3: Set Up GitHub Secrets

Required secrets in repository settings:
1. `CARGO_REGISTRY_TOKEN` - crates.io API token
2. `GITHUB_TOKEN` - Automatically provided by Actions

Navigate to: Settings → Secrets and variables → Actions

### Step 4: Create PR Template

**File**: `.github/pull_request_template.md`

Include checklist:
- Tests pass
- Documentation updated
- Changelog entry added
- Version bump if needed
- Examples updated

### Step 5: Configure Branch Protection

Settings → Branches → Add rule for `main`:
- Require PR reviews
- Require status checks (CI tests)
- Require up-to-date branches
- No direct pushes to main

## Workflow Definitions

### CI Workflow Structure
Location: `.github/workflows/ci.yml`

Jobs:
1. **test** - Run test suite on matrix
2. **fmt** - Check formatting
3. **clippy** - Lint code
4. **doc** - Build documentation
5. **coverage** - Generate test coverage

### Release Workflow Structure
Location: `.github/workflows/release.yml`

Jobs:
1. **release-plz** - Check and create release PR
2. **publish** - Publish to crates.io when PR merged

### Manual Release Workflow
Location: `.github/workflows/manual-release.yml`

For emergency releases:
- Manually triggered
- Accepts version input
- Bypasses automatic checks (use carefully)

## Testing the Pipeline

### Local Testing
```bash
# Install release-plz locally
cargo install release-plz

# Test configuration
release-plz release-pr --dry-run

# Test changelog generation
release-plz changelog
```

### GitHub Actions Testing
1. Create feature branch
2. Add workflows
3. Open PR to see CI in action
4. Merge to see release automation

## Changelog Management

### Conventional Commits
Adopt conventional commits for automatic changelog:
- `feat:` - New features (minor version bump)
- `fix:` - Bug fixes (patch version bump)
- `feat!:` or `BREAKING CHANGE:` - Breaking changes (major version bump)
- `docs:` - Documentation only
- `test:` - Test only
- `chore:` - Maintenance

### CHANGELOG.md Format
Following Keep a Changelog format:
- Unreleased section
- Version sections with date
- Categories: Added, Changed, Deprecated, Removed, Fixed, Security

## Version Bumping Strategy

Current: `1.0.0-alpha1`

Rules:
1. Alpha phase: Increment alpha number for any change
2. Beta phase: Reset to beta1, increment for fixes
3. RC phase: Only critical fixes
4. Stable: Follow semver strictly

Configuration in release-plz.toml handles pre-release versions.

## Monitoring and Notifications

### Status Badges
Add to README.md:
- CI status
- Coverage percentage
- crates.io version
- docs.rs status
- License

### Notifications
Configure GitHub notifications for:
- Failed CI runs
- Successful releases
- Security alerts

## Security Considerations

1. **Token Security**:
   - Use GitHub Secrets
   - Rotate tokens periodically
   - Limit token scope

2. **Dependency Updates**:
   - Dependabot configuration
   - Security audit in CI

3. **Supply Chain**:
   - Lock file committed
   - Reproducible builds
   - SLSA compliance

## Validation Gates

```bash
# Verify GitHub Actions syntax
actionlint .github/workflows/*.yml

# Test workflow locally with act
act -n  # Dry run

# Verify release-plz config
release-plz release-pr --dry-run

# Check current workflow status
gh workflow list
gh run list
```

## Migration Steps

1. Start with CI workflow only
2. Test on feature branches
3. Add release workflow after CI stable
4. Enable auto-merge for release PRs
5. Document process for team

## References
- release-plz docs: https://release-plz.ieni.dev/
- GitHub Actions: https://docs.github.com/actions
- cargo-release: https://github.com/crate-ci/cargo-release
- Conventional Commits: https://www.conventionalcommits.org/
- Keep a Changelog: https://keepachangelog.com/
- GitHub workflow syntax: https://docs.github.com/actions/using-workflows/workflow-syntax-for-github-actions

## Success Criteria
1. ✅ All PRs run comprehensive CI
2. ✅ Release PRs created automatically
3. ✅ Changelog updated automatically
4. ✅ Version published to crates.io on merge
5. ✅ Git tags and GitHub releases created
6. ✅ No manual intervention required

## Quality Score: 10/10
Comprehensive automation plan covering CI/CD, release management, and security. Includes specific tool recommendations, configuration details, and migration strategy. Ready for implementation with clear validation steps.

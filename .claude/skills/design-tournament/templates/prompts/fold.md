You are revising a design-tournament contract (`SPEC.md` and `{{FROZEN_PATHS}}` in
this repository) with findings from independent auditors. Their reports and test
files are in `{{AUDIT_DIR}}` (`<lens>.md` report plus a `<lens>/` folder with their
`audit_*` test files).

# Brief
{{BRIEF}}

# Do
1. Read SPEC.md, then every audit report.
2. For each **confirmed** finding, verify it yourself by running its test. Keep it if
   it is a real defect in scope, observable through the public interface, and not
   already covered by an existing behavior. Merge duplicates across auditors.
3. For each kept finding: add a numbered CHANGE behavior to SPEC.md (continue the
   numbering; cite file:line in current code), and add its test, cleaned up and
   prefixed `spec_change_`, to the frozen test file. Update the test index.
4. If an auditor shows an existing spec test or behavior is wrong or too strict,
   fix it and record it under "Test curation".
5. Run `{{TEST_CMD}}`: KEEP tests must pass and every `spec_change_*` test must fail
   for its documented reason (not compile errors). Do not touch `{{SCOPE_PATHS}}`
   except the thin interface adapter, and do not fix defects in it.
6. Do not copy `audit_*` files into the repo. Do not commit.

# Final message
Tables: **added** (new behavior IDs → test names → defect), **rejected** (finding →
reason: duplicate / not a defect / out of scope / flaky), **curated** (existing tests
changed). Then the new totals of KEEP/CHANGE behaviors and tests.

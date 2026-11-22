**Promises the project makes that the code does not keep.** Read README, docs/,
doc comments, error messages, function and test names in and around the scope. List
every concrete claim ("numerically stable", "validates", "symmetric", "returns X when
Y", "zero-copy", a test name asserting a property) and check each against the code,
writing a test where the claim is observable through the public interface. Tests that
cannot fail, or assert the opposite of their name, count as defects too.

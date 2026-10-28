# Contributing

## Setup

Clone the repository:

```
git clone https://github.com/leoS110/AUV_sim-control
```

In MATLAB, run `matlab/startup_auv1.m`. It adds the code folders to the path.

## Workflow

Use one branch per task. Start it from an up-to-date `main`:

```
git checkout main
git pull
git checkout -b <task-name>
```

- Make small commits. Start with a short summary line in the imperative
  ("Add bow thruster deadband"). Use the body to say why you made the change.
- Push your branch and open a pull request.
- One reviewer approves it. Then merge it on GitHub.
- Nobody commits directly to `main`.

## Keeping your branch up to date

```
git fetch origin
git merge origin/main
```

If two changes touch the same lines, Git stops and marks the file:

```
<<<<<<< HEAD
your version
=======
the version from main
>>>>>>> origin/main
```

Edit the file to keep what is right and delete the three marker lines. Then
run `git add <file>` and `git commit`. To back out and start again, run
`git merge --abort`.

Use merge, not rebase, on branches you have pushed.

## Before opening a pull request

Run these four tests (from anywhere, each sets up its own path):

```matlab
test_auv1_matches_remus100
test_auv1_actuator_signs
test_SIMauv1_regression
test_auv1Visualiser
```

CI runs these tests automatically on every pull request.

Rewrite the regression baseline with `test_SIMauv1_regression(true)` only when
you meant to change the behaviour. Say why in the pull request.

## Parameters

Every value in `matlab/auv1Params.m` carries its unit, source and uncertainty.
Mark a value `ESTIMATE` if nobody has measured it.

Every change to a parameter gets a row in `matlab/PARAMS_CHANGELOG.md`: date,
old → new, source and reason.

## External code and data

- Never edit `matlab/mss/` or `matlab/tests/reference/`.
- Copy an MSS function in unmodified, together with everything it calls.
- Keep the raw source files for any data, as in `matlab/data/thrusters/raw/`,
  together with the script that processes them.

# Frank3 Batch Simulation Manual

| | |
|---|---|
| **I** | Setup |
| **II** | Output |
| **III** | Code reference |

Ported from the Frank2 batch workflow, which is ported from the jawTongue workflow. The worker and batch folder live under `frank3`; the model is `FrankModel3` as an updated version of `FrankModel2`.

---

# I Setup

## 1 Prerequisites

- `artisynth_core` built → `$ARTISYNTH_HOME/classes`
- `artisynth_models` built, including `FrankModel3BatchWorker.java` → `artisynth_models/classes`
- Java 21 (x86_64)

## 2 Folder layout

```
src/artisynth/models/frank3/
├── FrankModel3.java
├── FrankModel3BatchWorker.java     # compiled into classes/
└── batch/
    ├── props.psl                   # set parameters here
    ├── bracingBatchDriver.py       # Jython driver
    ├── runSims.bash                # optional multi-worker template
    └── output/                     # auto-created by the worker
        ├── excitations.txt
        ├── position.txt
        └── failedexcitations.txt
```

`FrankActivations` (used by the worker to build probes) still lives in `frank2` and is imported from there — you do not need a copy in `frank3`.

## 3 Before running: two files to edit

### props.psl

The manager that defines the experiment parameters.

**Editing props.psl**

- **First time:** use the Frank / FrankMechModel paths, e.g.  
  `models/FrankMechModel/MuscleExciters/<Group>Exciters/<name>:excitation`.  
  Old jaw paths (`models/jawmodel/…`) and other non-Frank trees will not work.
- **Next times:** keep the paths the same. Just change the numbers inside `{ … }`.

Example (tongue exciters — same names as frank2):

```
"models/FrankMechModel/MuscleExciters/TongueExciters/GGP:excitation" = {%0% %0.1% %0.25%}
"models/FrankMechModel/MuscleExciters/TongueExciters/STY:excitation" = {%0% %0.1% %0.25%}
# ...one line per exciter you want to vary
```

Other inherited Frank groups use the same pattern with a different group folder, e.g.:

- `…/FaceExciters/…`
- `…/PalateExciters/…`
- `…/PharynxExciters/…`
- `…/LarynxExciters/…`
- `…/ExternalExciters/…`

**Note:** FrankModel3 also adds neck-muscle exciters under `mechModel` (`Exciter`, e.g. `longusColli_l`). Those are **not** included in `getAllExciters()` today, so this batch worker only sweeps the inherited Frank2-style exciters unless you extend the model/worker later.

### bracingBatchDriver.py

Starts the worker inside ArtiSynth:

```python
from artisynth.models.frank3 import FrankModel3BatchWorker
from jarray import array
import sys
args = array(sys.argv, String)
worker = FrankModel3BatchWorker(args)
worker.run()
```

**First time:** remember the import must be `FrankModel3BatchWorker`. If you copied the driver from frank2 or jaw-tongue, it may still import the old worker and will load the wrong one.

## 4 Generate your run commands

Choose your OS and enter your two repo paths. Trailing slashes are trimmed automatically.

| | |
|---|---|
| **artisynth_models path** | e.g. `E:\study\FolderOfArtisynth\artisynth_models` |
| **artisynth_core path** | e.g. `E:\study\FolderOfArtisynth\artisynth_core` |

### Environment — run in each terminal

**Windows · PowerShell** (example paths — edit to match your machine):

```powershell
cd E:\study\FolderOfArtisynth\artisynth_models\src\artisynth\models\frank3\batch
$env:JAVA_HOME = "C:\Program Files\Java\jdk-21"   # adjust
$env:PATH = "$env:JAVA_HOME\bin;$env:PATH"
$env:ARTISYNTH_HOME = "E:\study\FolderOfArtisynth\artisynth_core"
$env:CLASSPATH = "$env:ARTISYNTH_HOME\classes;E:\study\FolderOfArtisynth\artisynth_models\classes;$env:ARTISYNTH_HOME\lib\*"
```

**macOS · zsh** (example):

```bash
cd /path/to/artisynth_models/src/artisynth/models/frank3/batch
export JAVA_HOME=$(/usr/libexec/java_home -v 21 -a x86_64)
export PATH="$JAVA_HOME/bin:$PATH"
export ARTISYNTH_HOME=/path/to/artisynth_core
export CLASSPATH="$ARTISYNTH_HOME/classes:/path/to/artisynth_models/classes:$ARTISYNTH_HOME/lib/*"
```

### Terminal 1 — BatchManager

```bash
# serves tasks from props.psl
java artisynth.tools.batchsim.manager.BatchManager -f props.psl
```

### Terminal 2 — BatchWorker

```bash
# loads FrankModel3 and runs the worker
# -numSolverThreads: Pardiso OpenMP thread count. Try 1, 2, 4, 8 and pick the
# fastest for one task. Default (-1 / unset) often uses all cores and can be
# slower. With multiple workers, keep (workers) x (threads) <= physical cores.
java artisynth.core.driver.Main -noGui \
     -numSolverThreads 4 \
     -model artisynth.models.frank3.FrankModel3 \
     -script bracingBatchDriver.py
```

On Windows (cmd / PowerShell), the same ideas apply; use `;` in `CLASSPATH` and a single-line `java …` if line continuation is awkward.

**Optional:** `runSims.bash` starts one manager and several workers in the background (template). It already points at `FrankModel3` and passes `-numSolverThreads 2`; adjust worker count, thread count, and paths for your machine.

---

# II Output

Three CSV files in `batch/output/`. A task succeeds when the recorded FEMs reach static equilibrium within the settle window; otherwise it fails.

## excitations.txt

**Success · inputs**

One row per successful task — the excitation values that were applied.

```
taskCounter, val1, val2, …    e.g.  0,0.0,0.25
```

`valN` is the value assigned to the N-th swept exciter, in `props.psl` order.

## position.txt

**Success · result**

One row per FEM node, per successful task — the settled pose of the whole deformable anatomy.

```
taskCounter, femName, nodeNumber, x, y, z    e.g.  0,tongue,0,0.1162,-0.0241,0.0847
```

Node numbers restart at 0 per FEM — identify a node by the pair `(femName, nodeNumber)`.

Typical node counts (same meshes as frank2; confirm if your build differs):

| FEM | node numbers | nodes |
|-----|--------------|-------|
| tongue | 0 – 947 | 948 |
| face | 0 – 8719 | 8,720 |
| softPalate | 0 – 3180 | 3,181 |
| larynx | 0 – 3147 | 3,148 |
| pharynx | 0 – 2423 | 2,424 |
| **rows per task** | | **18,421** |

## failedexcitations.txt

**Failure · inputs**

One row per failed / unsettled task — same columns as `excitations.txt`.

```
taskCounter, val1, val2, …
```

## Which FEMs

The `record*` switches in the worker control which FEMs are written and which must settle for a task to count as successful. Turn ones you don't need off to shrink files and relax the settle test.

---

# III Code reference

| Tag | Meaning |
|-----|---------|
| **adapted** | ported from the frank2 / jaw-tongue worker |
| **new** | written for frank2, retained for frank3 |
| **frank3** | retargeted for `FrankModel3` / `frank3` package |

## Fields

| Field | Tag | Notes |
|-------|-----|--------|
| `myProbeDuration` | new | How long each exciter is held (cubic-step ramp then hold). Match it to the batch stop time. |
| `recordTongue` / `Face` / `SoftPalate` / `Pharynx` / `Larynx` | new | Per-FEM on/off. Controls both what is recorded and which FEMs must settle for success. |

## Methods

| Method | Tag | When | Role |
|--------|-----|------|------|
| constructor | frank3 | once | Casts the root to `FrankModel3`, grabs exciters via `getAllExciters()`, opens the three output files, clears existing input probes. |
| `initWriter(dir, name)` | adapted | | Opens a `PrintWriter` in append mode, creating `output/` if missing. |
| `preSim()` | frank3 | before each task | Refreshes the root reference, clears the previous task's probes, then builds this task's probes. |
| `addAllExciterProbes()` | adapted | | For each task property whose host is a `MuscleExciter`, reads the excitation the framework just applied and builds a cubic-step probe holding that value for `myProbeDuration`; adds it to the timeline. Uses `FrankActivations.createMuscleProbe` from frank2. |
| `removeAllExciterProbes()` | adapted | | Removes those probes (looked up by exciter name) so the next task starts clean. |
| `recordPosition()` | adapted | | Loops the enabled FEMs and writes their node positions. |
| `recordFemPositions(enabled, fem)` | new | | Helper — writes one line per node for a single FEM: `taskCounter, femName, nodeNumber, x, y, z`. |
| `recordSimResults()` | adapted | after each task | On success: `recordPosition()` + a row to `excitations.txt`. On failure: a row to `failedexcitations.txt`. |
| `postSim()` | adapted | after each task | Removes the exciter probes. |
| `setUpStopConditionMonitor()` | frank3 | defines success | Sets `myMaxTime` / `mySettleTime` and adds a `TimeChecker` (settle window) that nests an `EquilibriumChecker` requiring all enabled-FEM nodes to be static. Time gate is outer so node velocities are not scanned every step before the window. Not settled in time → task fails. |
| `addFemNodes(list, enabled, fem)` | new | | Helper — adds an enabled FEM's nodes to the equilibrium check. |
| `closeWriters()` | adapted | end of batch | Closes all three output files. |

---

*frank3 batch simulation · ArtiSynth BatchSim · adapted from frank2*

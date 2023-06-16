# Tongue bracing simulations

This repository is a fork of the `artisynth_models` repository. This branch contains code used to run the simulations in Study 3 from Liu et al. (2022).

The relevant files are contained in [`src/artisynth/models/jawTongue`](https://github.com/connormayer/artisynth_models/tree/bracing/src/artisynth/models/jawTongue). In particular:
* [`batch`](https://github.com/connormayer/artisynth_models/tree/bracing/src/artisynth/models/jawTongue/batch) contains the `props.pl` file that defines the muscle activations for each simulation, as well as some of the relevant BatchSim files.
* [`BadinJawHyoidTongueContactBatchWorker`](https://github.com/connormayer/artisynth_models/blob/bracing/src/artisynth/models/jawTongue/BadinJawHyoidTongueContactBatchWorker.java) defines how simulations and run and the results recorded.
* [`BadinJawHyoidTongueContact`](https://github.com/connormayer/artisynth_models/blob/bracing/src/artisynth/models/jawTongue/BadinJawHyoidTongueContact.java) is the model used for the simulations.

For the simulation results and their analysis, see [this repository](https://github.com/connormayer/bracing_simulations).

For more information on setting up Artisynth, see the [official website](https://www.artisynth.org). For more information on using BatchSim to run simulations in Artisynth, see [here](https://github.com/artisynth/artisynth_models/blob/master/doc/batchsim/batchsim.pdf).

## Citation

Liu, Y., Luo, S., Łuszczuk, M., Mayer, C., Shamei, A., de Boer, G., & Gick, B. (2022). [Robustness of lateral tongue bracing under bite block perturbation.](https://www.degruyter.com/document/doi/10.1515/phon-2022-0001/html) _Phonetica, 79_(6), 523-549. 

# Original `artisynth_models` README

This is the general distribution of ArtiSynth Models, a collection of
publicly available anatomical models created in ArtiSynth.

You need to have ArtiSynth (www.artisynth.org) installed to run the
models in this package.

Other files in this directory:

VERSION
    current version

LICENSE
    licensing and terms of use

Makefile
    Makefile for compiling and doing certain maintenance operations in
    a shell environment.

Makefile.base
    Base definitions for Makefile and Makefile in subdirectories.

doc
    documentation directory, just for javadocs noew

classes 
    root directory for compiled classes

src
    model source code

eclipseSettings.zip
    zip file containing Eclipse project settings

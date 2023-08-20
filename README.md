# IJRRCodeExamples

This repository is a companion to the paper "The Matroid Team Surviving Orienteers Problem and its Variants: Constrained Routing of Heterogeneous Teams with Risky Traversal".  The code here is intended to provide a more complete description of particular algorithms, but this is not a complete implementation of the algorithm presented in the paper.

## Efficient weight update

`efficient_weight_update.h` provides a concrete implementation of the efficient weight update routine from Section 5.3.1

## Lazy Subproblem Evaluation

`lazy_subproblem_evaluation.h` provides a concrete implementation of lazy subproblem evaluation as described in Section 6.2

## Partitioning Routines

`partitioning_routines` provides concrete implementations of various partitioning routines presented in Sections 4 and 5.1.3.  Different classes of matroids are organized into the same file to limit the size of the file.


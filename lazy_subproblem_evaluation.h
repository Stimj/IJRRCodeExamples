// @class Subproblem a subproblem type that is hashable
class Subproblem;

// @class MTSOProblem the MTSO instance to solve. Has a partition() method that returns a vector of subproblems
class MTSOProblem;

// @class SubproblemSolution a solution for a subproblem of the MTSO
class SubproblemSolution;

// @struct An upper bound struct which default initializes to max
struct UpperBound
{
  double value = std::numeric_limits::max();
};

/// Make a container so that we can track bounds on subproblems and order them for the priority queue ///
struct BoundedSubproblem
{
  Subproblem subproblem;
  UpperBound upper_bound;
};

operator<(const BoundedSubproblem& L, const BoundedSubproblem& R)
{
  return L.upper_bound < R.upper_bound;
}

/**
 * @brief LazySolveSubproblem solves problems in order of their upper bounds, and skips problems which
 * are 'dominated', meaning we know that they cannot contain the optimal solution for this round of the
 * greedy selection
 *
 * @param problem       The immutable MTSO problem to solve
 * @param upper_bounds  A mutable map of upper bounds for subproblems
 */
SubproblemSolution LazySolveSubproblem(const MTSOProblem& problem, std::unordered_map<Subproblem, UpperBound>& upper_bounds)
{
  std::queue<BoundedSubproblem> Q;

  // Initialize the queue
  for(const Subproblem& subproblem : problem.partition())
    Q.push(BoundedSubproblem{.subproblem=subproblem, .upper_bound=upper_bounds[subproblem]});


  SubproblemSolution best_solution;

  while(!Q.empty())
  {
    // If the best element left in the queue is guaranteed to be worse than the best solution we've found so far,
    // then we're done with this iteration of the greedy selection and do not need to evaluate any further subproblems
    if(Q.front().upper_bound < best_solution.objective_value)
    {
      break;
    }
    else
    {
      // The upper bound implies this subproblem might contain the solution to this greedy selection step
      SubproblemSolution new_solution = SolveSubproblem(Q.front().subproblem);

      // If this solution is the best yet, track it for later
      if(new_solution > best_solution)
        best_solution = new_solution;

      // Update the upper bound for this subproblem for future iterations
      upper_bounds[Q.front().subproblem] = new_solution.upper_bound;
    }
    Q.pop();
  }
  return best_solution;
}

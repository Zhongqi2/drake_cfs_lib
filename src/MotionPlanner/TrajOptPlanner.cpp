// Kinematic trajectory optimization (Drake) with collision constraints.
// Generalizes the original KinematicTrajOpt::solve_trajectory: same two-stage
// idea, but uses the shared collision checker, works for any number of joints,
// and falls back to an RRT-Connect seed when the straight-line seed fails.
#include <chrono>
#include <memory>
#include <optional>

#include "drake/common/trajectories/bspline_trajectory.h"
#include "drake/math/bspline_basis.h"
#include "drake/multibody/inverse_kinematics/minimum_distance_lower_bound_constraint.h"
#include "drake/planning/trajectory_optimization/kinematic_trajectory_optimization.h"
#include "drake/solvers/minimum_value_constraint.h"
#include "drake/solvers/ipopt_solver.h"
#include "drake/solvers/snopt_solver.h"

#include "Planners.h"

namespace motion_planner {

using drake::math::BsplineBasis;
using drake::planning::trajectory_optimization::KinematicTrajectoryOptimization;
using drake::trajectories::BsplineTrajectory;

namespace {

// B-spline whose control points are spread evenly (by arc length) along a
// joint-space polyline; used as the optimizer's initial guess.
BsplineTrajectory<double> GuessFromPath(const KinematicTrajectoryOptimization& trajopt,
                                        const std::vector<Eigen::VectorXd>& path,
                                        double duration) {
  std::vector<double> cum{0.0};
  for (size_t k = 1; k < path.size(); ++k) {
    cum.push_back(cum.back() + (path[k] - path[k - 1]).norm());
  }
  const double L = cum.back();
  const int N = trajopt.num_control_points();
  std::vector<drake::MatrixX<double>> ctrl;
  size_t seg = 0;
  for (int j = 0; j < N; ++j) {
    const double s = L * j / (N - 1);
    while (seg + 2 < path.size() && cum[seg + 1] < s) ++seg;
    const double len = cum[seg + 1] - cum[seg];
    const double a = len > 1e-12 ? std::clamp((s - cum[seg]) / len, 0.0, 1.0) : 0.0;
    ctrl.push_back((1 - a) * path[seg] + a * path[seg + 1]);
  }
  std::vector<double> knots = trajopt.basis().knots();
  for (double& k : knots) k *= duration;
  return BsplineTrajectory<double>(BsplineBasis<double>(trajopt.basis().order(), knots),
                                   ctrl);
}

}  // namespace

PlanningResult TrajOptPlanner::DoPlan(const PlanningRequest& req) const {
  PlanningResult result;
  const auto t_begin = std::chrono::steady_clock::now();
  const int n = scene().num_positions();
  const Eigen::VectorXd zero = Eigen::VectorXd::Zero(n);
  const Eigen::VectorXd& vmax = scene().velocity_limits();
  // The constraint keeps a pointer to this context; it must outlive Solve().
  auto checker_context = scene().checker().MakeStandaloneModelContext();
  // Margin on top of the checker padding, reduced if start/goal are already
  // closer than that to an obstacle (otherwise the problem is infeasible).
  const double bound = std::clamp(
      std::min(scene().MinDistance(req.start), scene().MinDistance(req.goal)) - 1e-3,
      0.0, params_.min_distance);
  auto collision_constraint =
      std::make_shared<drake::multibody::MinimumDistanceLowerBoundConstraint>(
          &scene().checker(), bound, checker_context.get(),
          drake::solvers::QuadraticallySmoothedHingeLoss,
          params_.influence_distance_offset);

  const drake::solvers::SnoptSolver snopt;
  const drake::solvers::IpoptSolver ipopt;
  const drake::solvers::SolverInterface* default_solver =
      snopt.available() && snopt.enabled() ? static_cast<const drake::solvers::SolverInterface*>(&snopt)
                                           : &ipopt;
  auto seconds_left = [&]() {
    return req.time_limit - std::chrono::duration<double>(
                                std::chrono::steady_clock::now() - t_begin).count();
  };

  // Iteration caps keep a single solve from running far past the time limit.
  auto options = [&]() {
    drake::solvers::SolverOptions o;
    o.SetOption(drake::solvers::SnoptSolver::id(), "Major iterations limit",
                params_.max_major_iterations);
    o.SetOption(drake::solvers::IpoptSolver::id(), "max_iter",
                params_.max_major_iterations);
    o.SetOption(drake::solvers::IpoptSolver::id(), "max_cpu_time",
                std::max(1.0, seconds_left()));
    return o;
  };

  // Builds and solves the problem from a seed path. If `presolve` is set, a
  // problem without collision constraints is solved first and used as the warm
  // start (the approach of the original KinematicTrajOpt). The result is
  // verified densely, since collision constraints only act at sample points.
  auto optimize = [&](const std::vector<Eigen::VectorXd>& seed, bool presolve,
                      const drake::solvers::SolverInterface& solver,
                      std::string* log) -> std::optional<BsplineTrajectory<double>> {
    KinematicTrajectoryOptimization trajopt(n, params_.num_control_points,
                                            params_.spline_order);
    trajopt.AddDurationCost(1.0);
    // Smooth "path energy" Σ‖c_{j+1} − c_j‖² instead of AddPathLengthCost():
    // the zero end-velocity constraints make c_0 = c_1, where the L2-norm
    // path-length cost is non-differentiable.
    const auto& cp = trajopt.control_points();
    for (int j = 0; j + 1 < cp.cols(); ++j) {
      trajopt.get_mutable_prog().AddQuadraticCost(
          (cp.col(j + 1) - cp.col(j)).cast<drake::symbolic::Expression>().squaredNorm());
    }
    trajopt.AddPositionBounds(scene().lower_limits(), scene().upper_limits());
    trajopt.AddVelocityBounds(-vmax, vmax);
    trajopt.AddDurationConstraint(params_.min_duration, params_.max_duration);
    trajopt.AddPathPositionConstraint(req.start, req.start, 0);
    trajopt.AddPathPositionConstraint(req.goal, req.goal, 1);
    trajopt.AddPathVelocityConstraint(zero, zero, 0);
    trajopt.AddPathVelocityConstraint(zero, zero, 1);
    double T_guess = 0;
    for (size_t k = 1; k < seed.size(); ++k) {
      T_guess += QuinticDuration(seed[k - 1], seed[k], vmax, 0.0);
    }
    T_guess = std::clamp(T_guess, params_.min_duration, params_.max_duration);
    trajopt.SetInitialGuess(GuessFromPath(trajopt, seed, T_guess));

    if (presolve) {
      drake::solvers::MathematicalProgramResult pre;
      solver.Solve(trajopt.prog(), std::nullopt, options(), &pre);
      if (pre.is_success()) trajopt.SetInitialGuess(trajopt.ReconstructTrajectory(pre));
    }
    for (int i = 0; i < params_.num_collision_samples; ++i) {
      const double s = static_cast<double>(i) / (params_.num_collision_samples - 1);
      trajopt.AddPathPositionConstraint(collision_constraint, s);
    }
    drake::solvers::MathematicalProgramResult final_result;
    solver.Solve(trajopt.prog(), std::nullopt, options(), &final_result);
    *log += solver.solver_id().name() + ": ";
    if (!final_result.is_success()) {
      *log += drake::solvers::to_string(final_result.get_solution_result());
      if (final_result.get_solver_id() == drake::solvers::SnoptSolver::id()) {
        *log += " (info " +
                std::to_string(final_result.get_solver_details<drake::solvers::SnoptSolver>().info) +
                ")";
      }
      *log += "; ";
      return std::nullopt;
    }
    // Bug fix vs. the original KinematicTrajOpt: reconstruct from the
    // *final* (collision-constrained) solve, not the first one.
    auto traj = trajopt.ReconstructTrajectory(final_result);
    if (!IsTrajectoryValid(scene(), traj)) {
      *log += "solved but clips an obstacle between samples; ";
      return std::nullopt;
    }
    *log += "ok; ";
    return traj;
  };

  // Attempt cascade: SNOPT and IPOPT tend to fail on *different* queries, and
  // different RRT seeds lead to different local optima.
  // A straight seed through an obstacle mostly ends in a poor local minimum,
  // so it is only used when the straight line is already collision-free.
  std::string log;
  std::optional<BsplineTrajectory<double>> traj;
  if (scene().IsEdgeCollisionFree(req.start, req.goal)) {
    log = "straight seed → ";
    traj = optimize({req.start, req.goal}, /*presolve=*/true, *default_solver, &log);
  }

  if (params_.use_rrt_seed_fallback) {
    RRTConnectPlanner rrt(scene());
    std::mt19937 rng(req.seed ? req.seed : std::random_device{}());
    for (int round = 0; round < params_.max_rrt_seeds && !traj && seconds_left() > 0;
         ++round) {
      std::string rrt_info;
      auto path = rrt.FindPath(req.start, req.goal, seconds_left(), &rng, &rrt_info);
      if (path.empty()) {
        log += "RRT seed failed (" + rrt_info + "); ";
        break;
      }
      path = rrt.Shortcut(std::move(path), &rng);
      result.waypoints = path;
      for (const drake::solvers::SolverInterface* solver :
           {default_solver, static_cast<const drake::solvers::SolverInterface*>(&ipopt)}) {
        if (traj || seconds_left() <= 0) break;
        if (solver == &ipopt && default_solver == &ipopt) continue;
        log += "RRT seed " + std::to_string(round + 1) + " → ";
        traj = optimize(path, /*presolve=*/false, *solver, &log);
      }
    }
  }

  result.message = log;
  if (traj) {
    result.trajectory = std::make_shared<BsplineTrajectory<double>>(*traj);
    result.success = true;
  }
  return result;
}

}  // namespace motion_planner

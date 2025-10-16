#pragma once
#include <iostream>
#include <Eigen/Dense>
#include <chrono>

static constexpr double EQ_THR = 1e-6;

template <int NX, int NU, int NY, int NC, int NH,
          auto f, auto fx, auto fu,
          auto L, auto Lx, auto Lu, auto Lxx, auto Luu, auto Lux,
          auto L_ter, auto L_terx, auto L_terxx,
          auto h_ter, auto h_terx,
          auto g, auto gx, auto gu>
class DdpSolver
{
public:
  int max_iters;
  double alpha_0, alpha_converge_threshold, line_search_decrease_factor;
  
  // trajectories
  std::array<Eigen::Vector<double, NX>, NH+1> x, x_new, x_guess, d, d_new;
  std::array<Eigen::Vector<double, NU>, NH> u, u_new, u_guess;
  std::array<Eigen::Vector<double, NC>, NH+1> λ, λ_new;
  Eigen::Vector<double, NX> x_0;
  
  // dynamics
  Eigen::Matrix<double, NX, NX> fx_eval;
  Eigen::Matrix<double, NX, NU> fu_eval;

  // cost
  Eigen::Vector<double, NX> lx_eval, Qx;
  Eigen::Vector<double, NU> lu_eval, Qu;
  Eigen::Matrix<double, NX, NX> lxx_eval, Qxx;
  Eigen::Matrix<double, NU, NU> luu_eval, Quu, Quu_inv, I;
  Eigen::Matrix<double, NU, NX> lux_eval, Qux;

  // riccati gains
  std::array<Eigen::Matrix<double, NU, 1>, NH> k;
  std::array<Eigen::Matrix<double, NU, NX>, NH> K;
  
  // constraints
  Eigen::Vector<double, NY> λter;
  Eigen::Vector<double, NC> g_eval;
  Eigen::Matrix<double, NC, NX> gx_eval;
  Eigen::Matrix<double, NC, NU> gu_eval;
  Eigen::Vector<double, NY> h_ter_eval;
  Eigen::Matrix<double, NY, NX> h_terx_eval;
  double μ;
  Eigen::Vector<double, NC> Iμ;

  // value function approximation
  Eigen::Vector<double, NX> Vx;
  Eigen::Matrix<double, NX, NX> Vxx;

public:
  DdpSolver(int max_iters = 10,
            double alpha_0 = 1.0,
            double alpha_converge_threshold = 1e-1,
            double line_search_decrease_factor = 0.5)
    : max_iters(max_iters), alpha_0(alpha_0),
      alpha_converge_threshold(alpha_converge_threshold), line_search_decrease_factor(line_search_decrease_factor)
  {
    // identity matrix for computing inverse
    I = Eigen::Matrix<double, NU, NU>::Identity(NU, NU);

    // initialize trajectories
    for (int i = 0; i < NH; ++i)
    {
      x_guess[i].setZero();
      u_guess[i].setZero();
      λ[i].setZero();
      λ_new[i].setZero();
    }
    x_guess[NH].setZero();
    λ[NH].setZero();
    λ_new[NH].setZero();
    x_0.setZero();
    λter.setZero();
    μ = 1000.0;
    Iμ = Eigen::Matrix<double, NC, 1>::Ones();

    d[0].setZero();
    d_new[0].setZero();
  }

  double compute_cost(
      const std::array<Eigen::Vector<double, NX>, NH+1>& x, 
      const std::array<Eigen::Vector<double, NU>, NH>& u)
  {
    double cost = 0.0;
    // running cost
    for (int i = 0; i < NH; ++i)
      cost += L(x[i], u[i], i)(0,0);
    return cost + L_ter(x[NH])(0,0);
  }

  double compute_penalty(
      const std::array<Eigen::Vector<double, NX>, NH+1>& x,
      const std::array<Eigen::Vector<double, NU>, NH>& u,
      const std::array<Eigen::Vector<double, NX>, NH+1>& d)
  {
    double penalty = 0.0;
    // inequality constraints
    if constexpr (g)
    {
      for (int i = 0; i < NH; ++i)
      {
        g_eval = g(x[i], i);
        for (int j = 0; j < NC; ++j)
          Iμ(j,0) = (g_eval(j) < 0.0 && λ[i](j) < EQ_THR) ? 0.0 : μ;
        penalty += 0.5 * g_eval.transpose() * Iμ.asDiagonal() * g_eval + 0.5 * μ * d[i].squaredNorm();
      }
      g_eval = g(x[NH], NH);
      for (int j = 0; j < NC; ++j)
      Iμ(j,0) = (g_eval(j) < 0.0 && λ[NH](j) < EQ_THR) ? 0.0 : μ;
      penalty += 0.5 * g_eval.transpose() * Iμ.asDiagonal() * g_eval;
    }

    // terminal equality constraint
    if constexpr (h_ter)
    {
      h_ter_eval = h_ter(x[NH]);
      penalty += 0.5 * μ * h_ter_eval.squaredNorm();
    }

    penalty += 0.5 * μ * d[NH].squaredNorm();
    return penalty;
  }

  void backward_pass()
  {
    // initialize value function with terminal cost approximation
    Vx = L_terx(x[NH]);
    Vxx = L_terxx(x[NH]);

    // evaluate constraints at terminal state
    if constexpr (g)
    {
      g_eval = g(x[NH], NH);
      gx_eval = gx(x[NH], NH);
      for (int j = 0; j < NC; ++j)
        Iμ(j,0) = (g_eval(j) < 0.0 && λ[NH](j) < EQ_THR) ? 0.0 : μ;
      Vx += gx_eval.transpose() * (λ[NH] + Iμ.asDiagonal() * g_eval);
      Vxx += gx_eval.transpose() * Iμ.asDiagonal() * gx_eval;
    }

    if constexpr (h_ter)
    {
      h_ter_eval = h_ter(x[NH]);
      h_terx_eval = h_terx(x[NH]);
      Vx += h_terx_eval.transpose() * (λter + μ * h_ter_eval);
      Vxx += μ * h_terx_eval.transpose() * h_terx_eval;
    }
                                        
    for (int i = NH-1; i >= 0; --i)
    {
      // evaluate dynamics
      fx_eval = fx(x[i], u[i]);
      fu_eval = fu(x[i], u[i]);

      // evaluate cost
      lx_eval  = Lx (x[i], u[i], i);
      lu_eval  = Lu (x[i], u[i], i);
      lxx_eval = Lxx(x[i], u[i], i);
      luu_eval = Luu(x[i], u[i], i);
      lux_eval = Lux(x[i], u[i], i);

      // Q-function approximation
      Qx.noalias() = lx_eval + fx_eval.transpose() * (Vx + Vxx * d[i+1]);
      Qu.noalias() = lu_eval + fu_eval.transpose() * (Vx + Vxx * d[i+1]);
      Qxx.noalias() = lxx_eval + fx_eval.transpose() * Vxx * fx_eval;
      Quu.noalias() = luu_eval + fu_eval.transpose() * Vxx * fu_eval;
      Qux.noalias() = lux_eval + fu_eval.transpose() * Vxx * fx_eval;

      // add constraint contributions if present
      if constexpr (g)
      {
        g_eval = g(x[i], i);
        gx_eval = gx(x[i], i);
        gu_eval = gu(x[i], i);
        for (int j = 0; j < NC; ++j)
          Iμ(j,0) = (g_eval(j) < 0.0 && λ[i](j) < EQ_THR) ? 0.0 : μ;

        Qx.noalias() += gx_eval.transpose() * (λ[i] + Iμ.asDiagonal() * g_eval);
        Qu.noalias() += gu_eval.transpose() * (λ[i] + Iμ.asDiagonal() * g_eval);
        Qxx.noalias() += gx_eval.transpose() * Iμ.asDiagonal() * gx_eval;
        Quu.noalias() += gu_eval.transpose() * Iμ.asDiagonal() * gu_eval;
        Qux.noalias() += gu_eval.transpose() * Iμ.asDiagonal() * gx_eval;

      }

      // optimal gains
      Quu_inv = Quu.ldlt().solve(I);
      k[i].noalias() = - Quu_inv * Qu;
      K[i].noalias() = - Quu_inv * Qux;

      // update value function approximation
      Vx.noalias() = Qx - K[i].transpose() * Quu * k[i];
      Vxx.noalias() = Qxx - K[i].transpose() * Quu * K[i];
    }
  }

  void line_search()
  {
    bool converged = false;  // for the moment we don't have a convergence check
    bool sufficient_decrease = false;
    x_new[0] = x[0];
    double alpha = alpha_0;
    double cost = compute_cost(x, u) + compute_penalty(x, u, d);
    double cost_new = cost;
    while (true)
    {
      // forward pass
      for (int i = 0; i < NH; ++i)
      {
        u_new[i] = u[i] + alpha * k[i] + K[i] * (x_new[i] - x[i]);
        x_new[i+1] = f(x_new[i], u_new[i]) - (1.0 - alpha) * d[i+1];
        d_new[i+1] = (1.0 - alpha) * d[i+1];
      }

      cost_new = compute_cost(x_new, u_new) + compute_penalty(x_new, u_new, d_new);
      if (cost_new < cost)
      {
        // accept step
        x = x_new;
        u = u_new;
        d = d_new;
        
        // update multipliers
        if constexpr (g)
        {
          for (int i = 0; i < NH+1; ++i)
            λ[i] = (λ[i] + alpha * μ * g(x_new[i], i)).cwiseMax(0.0);
        }
        if constexpr (h_ter)
          λter = λter + alpha * μ * h_ter(x_new[NH]);

        // allow to break out of the loop
        sufficient_decrease = true;
      } 
      else
      {
        // reject step and decrease step size
        alpha *= line_search_decrease_factor;
        if (alpha < alpha_converge_threshold)
        {
          sufficient_decrease = true;
        }
      }
      if (sufficient_decrease) break;
    }
    μ *= 2.0;
  }

  void solve()
  {
    auto start_time = std::chrono::high_resolution_clock::now();

    // initial forward pass
    x = x_guess;
    u = u_guess;
    x[0] = x_0;
    for (int i = 0; i < NH; ++i)
      d[i+1].noalias() = f(x[i], u[i]) - x[i+1];

    for (int iter = 0; iter < max_iters; ++iter)
    {
      // backward pass
      auto start = std::chrono::high_resolution_clock::now();
      backward_pass();
      auto end = std::chrono::high_resolution_clock::now();
      std::chrono::duration<double> elapsed = (end - start) * 1000;
      std::cout << "Backward Pass: " << (std::chrono::high_resolution_clock::now() - start).count() * 1000 << " ms" << std::endl;

      // line search
      start = std::chrono::high_resolution_clock::now();
      line_search();
      end = std::chrono::high_resolution_clock::now();
      elapsed = (end - start) * 1000;
      std::cout << "Forward Pass: " << elapsed.count() << " ms" << std::endl;
    }

    auto end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = (end_time - start_time) * 1000;
    std::cout << "Total Time: " << elapsed.count() << " ms" << std::endl;
  }

  // setters
  void set_x_warmstart(const std::array<Eigen::Vector<double, NX>, NH+1>& xs) { x_guess = xs; }
  void set_u_warmstart(const std::array<Eigen::Vector<double, NU>, NH>& us) { u_guess = us; }
  void set_initial_state(const Eigen::Matrix<double, NX, 1>& x0) { x_0 = x0; }
};

#pragma once
#include <iostream>
#include <Eigen/Dense>
#include <chrono>

static constexpr double EQ_THR = 1e-6;

template <int NX, int NU, int NY, int NC, int NH>
class DdpSolver
{
public:

  // convenient aliases
  using VectorX = Eigen::Matrix<double, NX, 1>;
  using VectorU = Eigen::Matrix<double, NU, 1>;
  using VectorY = Eigen::Matrix<double, NY, 1>;
  using VectorC = Eigen::Matrix<double, NC, 1>;
  using MatrixXX = Eigen::Matrix<double, NX, NX>;
  using MatrixXU = Eigen::Matrix<double, NX, NU>;
  using MatrixUX = Eigen::Matrix<double, NU, NX>;
  using MatrixUU = Eigen::Matrix<double, NU, NU>;
  using MatrixCX = Eigen::Matrix<double, NC, NX>;
  using MatrixCU = Eigen::Matrix<double, NC, NU>;
  using MatrixHY = Eigen::Matrix<double, NY, NX>;

  // function types (adjust signatures if your functions differ)
  using FuncF    = std::function<VectorX(const VectorX&, const VectorU&)>;
  using FuncFX   = std::function<MatrixXX(const VectorX&, const VectorU&)>;
  using FuncFU   = std::function<MatrixXU(const VectorX&, const VectorU&)>;
  using FuncL    = std::function<Eigen::Matrix<double, 1, 1>(const VectorX&, const VectorU&, int)>;
  using FuncLx   = std::function<VectorX(const VectorX&, const VectorU&, int)>;
  using FuncLu   = std::function<VectorU(const VectorX&, const VectorU&, int)>;
  using FuncLxx  = std::function<MatrixXX(const VectorX&, const VectorU&, int)>;
  using FuncLuu  = std::function<MatrixUU(const VectorX&, const VectorU&, int)>;
  using FuncLux  = std::function<MatrixUX(const VectorX&, const VectorU&, int)>;
  using FuncLter = std::function<Eigen::Matrix<double, 1, 1>(const VectorX&)>;
  using FuncLterx= std::function<VectorX(const VectorX&)>;
  using FuncLterxx = std::function<MatrixXX(const VectorX&)>;
  using FuncHter   = std::function<VectorY(const VectorX&)>;
  using FuncHterx  = std::function<MatrixHY(const VectorX&)>;
  using FuncG      = std::function<VectorC(const VectorX&, int)>;
  using FuncGx     = std::function<MatrixCX(const VectorX&, int)>;
  using FuncGu     = std::function<MatrixCU(const VectorX&, int)>;

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

private:
  // store callables as members (runtime)
  FuncF f_;
  FuncFX fx_;
  FuncFU fu_;
  FuncL L_;
  FuncLx Lx_;
  FuncLu Lu_;
  FuncLxx Lxx_;
  FuncLuu Luu_;
  FuncLux Lux_;
  FuncLter L_ter_;
  FuncLterx L_terx_;
  FuncLterxx L_terxx_;
  FuncHter h_ter_;
  FuncHterx h_terx_;
  FuncG g_;
  FuncGx gx_;
  FuncGu gu_;

public:
  DdpSolver(FuncF f, FuncFX fx, FuncFU fu,
            FuncL L, FuncLx Lx, FuncLu Lu, FuncLxx Lxx, FuncLuu Luu, FuncLux Lux,
            FuncLter L_ter, FuncLterx L_terx, FuncLterxx L_terxx,
            FuncHter h_ter, FuncHterx h_terx,
            FuncG g, FuncGx gx, FuncGu gu,
            int max_iters = 10,
            double alpha_0 = 1.0,
            double alpha_converge_threshold = 1e-1,
            double line_search_decrease_factor = 0.5)
    : f_(std::move(f)), fx_(std::move(fx)), fu_(std::move(fu)),
      L_(std::move(L)), Lx_(std::move(Lx)), Lu_(std::move(Lu)), Lxx_(std::move(Lxx)), Luu_(std::move(Luu)), Lux_(std::move(Lux)),
      L_ter_(std::move(L_ter)), L_terx_(std::move(L_terx)), L_terxx_(std::move(L_terxx)),
      h_ter_(std::move(h_ter)), h_terx_(std::move(h_terx)),
      g_(std::move(g)), gx_(std::move(gx)), gu_(std::move(gu)),
      max_iters(max_iters), alpha_0(alpha_0),
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
      cost += L_(x[i], u[i], i)(0,0);
    return cost + L_ter_(x[NH])(0,0);
  }

  double compute_penalty(
      const std::array<Eigen::Vector<double, NX>, NH+1>& x,
      const std::array<Eigen::Vector<double, NU>, NH>& u,
      const std::array<Eigen::Vector<double, NX>, NH+1>& d)
  {
    double penalty = 0.0;
    // inequality constraints
    if (g_)
    {
      for (int i = 0; i < NH; ++i)
      {
        g_eval = g_(x[i], i);
        for (int j = 0; j < NC; ++j)
          Iμ(j,0) = (g_eval(j) < 0.0 && λ[i](j) < EQ_THR) ? 0.0 : μ;
        penalty += 0.5 * g_eval.transpose() * Iμ.asDiagonal() * g_eval + 0.5 * μ * d[i].squaredNorm();
      }
      g_eval = g_(x[NH], NH);
      for (int j = 0; j < NC; ++j)
      Iμ(j,0) = (g_eval(j) < 0.0 && λ[NH](j) < EQ_THR) ? 0.0 : μ;
      penalty += 0.5 * g_eval.transpose() * Iμ.asDiagonal() * g_eval;
    }

    // terminal equality constraint
    if (h_ter_)
    {
      h_ter_eval = h_ter_(x[NH]);
      penalty += 0.5 * μ * h_ter_eval.squaredNorm();
    }

    penalty += 0.5 * μ * d[NH].squaredNorm();
    return penalty;
  }

  void backward_pass()
  {
    // initialize value function with terminal cost approximation
    Vx = L_terx_(x[NH]);
    Vxx = L_terxx_(x[NH]);

    // evaluate constraints at terminal state
    if (g_)
    {
      g_eval = g_(x[NH], NH);
      gx_eval = gx_(x[NH], NH);
      for (int j = 0; j < NC; ++j)
        Iμ(j,0) = (g_eval(j) < 0.0 && λ[NH](j) < EQ_THR) ? 0.0 : μ;
      Vx += gx_eval.transpose() * (λ[NH] + Iμ.asDiagonal() * g_eval);
      Vxx += gx_eval.transpose() * Iμ.asDiagonal() * gx_eval;
    }

    if (h_ter_)
    {
      h_ter_eval = h_ter_(x[NH]);
      h_terx_eval = h_terx_(x[NH]);
      Vx += h_terx_eval.transpose() * (λter + μ * h_ter_eval);
      Vxx += μ * h_terx_eval.transpose() * h_terx_eval;
    }
                                        
    for (int i = NH-1; i >= 0; --i)
    {
      // evaluate dynamics
      fx_eval = fx_(x[i], u[i]);
      fu_eval = fu_(x[i], u[i]);

      // evaluate cost
      lx_eval  = Lx_ (x[i], u[i], i);
      lu_eval  = Lu_ (x[i], u[i], i);
      lxx_eval = Lxx_(x[i], u[i], i);
      luu_eval = Luu_(x[i], u[i], i);
      lux_eval = Lux_(x[i], u[i], i);

      // Q-function approximation
      Qx.noalias() = lx_eval + fx_eval.transpose() * (Vx + Vxx * d[i+1]);
      Qu.noalias() = lu_eval + fu_eval.transpose() * (Vx + Vxx * d[i+1]);
      Qxx.noalias() = lxx_eval + fx_eval.transpose() * Vxx * fx_eval;
      Quu.noalias() = luu_eval + fu_eval.transpose() * Vxx * fu_eval;
      Qux.noalias() = lux_eval + fu_eval.transpose() * Vxx * fx_eval;

      // add constraint contributions if present
      if (g_)
      {
        g_eval = g_(x[i], i);
        gx_eval = gx_(x[i], i);
        gu_eval = gu_(x[i], i);
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
        x_new[i+1] = f_(x_new[i], u_new[i]) - (1.0 - alpha) * d[i+1];
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
        if (g_)
        {
          for (int i = 0; i < NH+1; ++i)
            λ[i] = (λ[i] + alpha * μ * g_(x_new[i], i)).cwiseMax(0.0);
        }
        if (h_ter_)
          λter = λter + alpha * μ * h_ter_(x_new[NH]);

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
      d[i+1].noalias() = f_(x[i], u[i]) - x[i+1];

    for (int iter = 0; iter < max_iters; ++iter)
    {
      // backward pass
      // auto start = std::chrono::high_resolution_clock::now();
      backward_pass();
      // auto end = std::chrono::high_resolution_clock::now();
      // std::chrono::duration<double> elapsed = (end - start) * 1000;
      // std::cout << "Backward Pass: " << (std::chrono::high_resolution_clock::now() - start).count() * 1000 << " ms" << std::endl;

      // line search
      // start = std::chrono::high_resolution_clock::now();
      line_search();
      // end = std::chrono::high_resolution_clock::now();
      // elapsed = (end - start) * 1000;
      // std::cout << "Forward Pass: " << elapsed.count() << " ms" << std::endl;
    }

    auto end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = (end_time - start_time) * 1000;
    std::cout << "DDP Total Time: " << elapsed.count() << " ms" << std::endl;
  }

  // setters
  void set_x_warmstart(const std::array<Eigen::Vector<double, NX>, NH+1>& xs) { x_guess = xs; }
  void set_u_warmstart(const std::array<Eigen::Vector<double, NU>, NH>& us) { u_guess = us; }
  void set_initial_state(const Eigen::Matrix<double, NX, 1>& x0) { x_0 = x0; }
};

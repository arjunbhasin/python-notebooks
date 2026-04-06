# Finance Notebooks — Enhancement Plan

## Goal

Every notebook should be **theory-centric and beginner-friendly** (CFA-L1 style where applicable). Target: markdown-to-code ratio of at least **2.0** — theory dominates, code serves as illustration.

Each notebook should include:
- Intuitive plain-English explanations before every formula
- Step-by-step worked examples with actual numbers
- `> **Key Concept:**` boxes, `> **CFA Exam Tip:**`, `> **Common Mistake:**` callouts
- Transition paragraphs before every code cell, interpretation after every code cell

---

## Current Status (as of 2026-04-05)

### Done (ratio >= 2.0) — 24 notebooks

| Notebook | Cells | MD lines | Code lines | Ratio |
|:---------|:-----:|:--------:|:----------:|:-----:|
| portfolio-theory/diversification | 42 | 836 | 320 | 2.61 |
| fixed-income/bond-valuation | 36 | 487 | 226 | 2.15 |
| option-pricing/implied-volatility | 33 | 640 | 309 | 2.07 |
| fixed-income/credit-risk | 39 | 592 | 294 | 2.01 |
| option-pricing/monte-carlo-pricing | 62 | 751 | 374 | 2.01 |
| FSA/balance-sheet-working-capital | 31 | 730 | 364 | 2.01 |
| stochastic-processes/GBM | 40 | 629 | 314 | 2.00 |
| quantitative-methods/sampling-estimation | 42 | 949 | 474 | 2.00 |
| quantitative-methods/linear-regression | 55 | 1029 | 514 | 2.00 |
| mathematics-of-finance/mortgage-amortization | 43 | 1172 | 586 | 2.00 |
| mathematics-of-finance/yield-curves | 37 | 714 | 357 | 2.00 |
| portfolio-theory/risk-return | 46 | 792 | 396 | 2.00 |
| portfolio-theory/mean-variance | 32 | 536 | 268 | 2.00 |
| FSA/intro-financial-statements | 28 | 664 | 332 | 2.00 |
| FSA/income-statement-analysis | 37 | 846 | 424 | 2.00 |
| FSA/earnings-quality | 39 | 1015 | 508 | 2.00 |
| numerical-methods/lattice-methods | 43 | 1035 | 518 | 2.00 |
| mathematics-of-finance/interest-rate-math | 36 | 759 | 380 | 2.00 |
| portfolio-theory/capm-factor-models | 37 | 707 | 354 | 2.00 |
| fixed-income/interest-rate-risk | 39 | 593 | 297 | 2.00 |
| option-pricing/put-call-parity | 37 | 840 | 421 | 2.00 |
| numerical-methods/finite-difference | 41 | 838 | 420 | 2.00 |

### Remaining (ratio < 2.0) — 9 notebooks

#### Medium effort (ratio 1.0–1.5) — need moderate additions

| Notebook | Ratio | MD lines to add |
|:---------|:-----:|:---------------:|
| quantitative-methods/time-value-of-money | 1.31 | ~285 |
| option-pricing/binomial-model | 1.25 | ~295 |
| option-pricing/black-scholes-merton | 1.08 | ~288 |
| quantitative-methods/statistical-concepts | 1.01 | ~399 |

#### Heavy lift (ratio < 1.0) — need substantial work

| Notebook | Ratio | MD lines to add |
|:---------|:-----:|:---------------:|
| FSA/cash-flow-analysis | 0.91 | ~701 |
| mathematics-of-finance/bond-math | 0.91 | ~496 |
| quantitative-methods/probability-bayes | 0.90 | ~455 |
| stochastic-processes/mean-reversion | 0.81 | ~393 |
| FSA/ratio-analysis-dupont | 0.63 | ~974 |
| FSA/integrated-modeling | 0.44 | ~1,664 |
| stochastic-processes/jump-diffusion | 0.31 | ~993 |

---

## Suggested Batch Order (remaining work)

| Batch | Notebooks | Total MD to add |
|:------|:----------|:---------------:|
| **Batch A** | TVM, binomial-model, BSM | ~868 |
| **Batch B** | statistical-concepts, bond-math, probability-bayes | ~1,350 |
| **Batch C** | mean-reversion, jump-diffusion | ~1,386 |
| **Batch D** | FSA: cash-flow, ratio-analysis, integrated-modeling | ~3,339 |

**Total remaining: ~6,943 markdown lines across 11 notebooks.**

---

## Approach

For each notebook:
1. Read the existing notebook
2. Keep all code cells unchanged
3. Expand existing markdown cells + insert new ones between code cells
4. Validate JSON and confirm ratio >= 2.0

---

## Full Notebook Inventory (33 total)

### Quantitative Methods (CFA-L1) — 5 notebooks
- `time-value-of-money/` — PV, FV, annuities, perpetuities, EAR, loan amortization
- `statistical-concepts/` — Descriptive stats, distributions, hypothesis testing
- `probability-bayes/` — Bayes' theorem, conditional probability, expected value
- `sampling-estimation/` — CLT, confidence intervals, bootstrap
- `linear-regression/` — OLS, R-squared, ANOVA, CAPM beta estimation

### Mathematics of Finance — 4 notebooks
- `interest-rate-math/` — Compounding, rate conversions, forward rates, day-count conventions
- `bond-math/` — Duration, convexity, immunization
- `yield-curves/` — Bootstrapping, Nelson-Siegel, Svensson, term structure theories
- `mortgage-amortization/` — Fixed-rate, ARM, prepayment, refinancing, WAL

### Option Pricing — 5 notebooks
- `binomial-model/` — CRR tree, risk-neutral pricing, American options, convergence
- `black-scholes-merton/` — GBM, Ito's lemma, BSM formula, Greeks, delta hedging
- `monte-carlo-pricing/` — Risk-neutral MC, variance reduction, Asian/barrier options
- `implied-volatility/` — Newton-Raphson, bisection, vol smile/surface, VIX
- `put-call-parity/` — Parity derivation, arbitrage, bounds, synthetics

### Fixed Income (CFA-L1) — 3 notebooks
- `bond-valuation/` — Coupon bond pricing, YTM, clean/dirty price
- `credit-risk/` — Merton model, transition matrices, CDS basics
- `interest-rate-risk/` — Duration, DV01, convexity, key rate duration, immunization

### Portfolio Theory (CFA-L1) — 4 notebooks
- `mean-variance/` — Markowitz, efficient frontier, two-fund theorem
- `capm-factor-models/` — CML, SML, beta, Fama-French 3-factor
- `risk-return/` — Sharpe, Sortino, Treynor, VaR, CVaR, MWR/TWR, utility functions
- `diversification/` — Correlation, risk budgeting, ERC, international diversification

### Stochastic Processes — 3 notebooks
- `geometric-brownian-motion/` — Wiener process, Ito's lemma, GBM simulation, calibration
- `mean-reversion/` — OU process, Vasicek, CIR, affine term structure
- `jump-diffusion/` — Poisson process, Merton model, option pricing under jumps

### Numerical Methods in Finance — 2 notebooks
- `finite-difference/` — Explicit, implicit, Crank-Nicolson, American options
- `lattice-methods/` — Trinomial trees, barrier options, Hull-White tree

### Financial Statement Analysis (CFA-L1) — 7 notebooks
- `intro-financial-statements/` — Three core statements, accounting equation, accrual vs cash, IFRS vs GAAP
- `income-statement-analysis/` — Multi-step format, EPS, common-size analysis, margin trends
- `balance-sheet-working-capital/` — Inventory methods (FIFO/LIFO/WA), depreciation, cash conversion cycle
- `cash-flow-analysis/` — Direct vs indirect method, FCFF/FCFE, lifecycle stage analysis
- `ratio-analysis-dupont/` — 5 ratio categories, DuPont 3-factor & 5-factor, cross-sectional analysis
- `earnings-quality/` — Accrual anomaly, Beneish M-Score, Jones model
- `integrated-modeling/` — 3-statement model, circular reference resolution, scenario analysis

---

## Enhancement Log

| Date | Notebook | What was added |
|:-----|:---------|:---------------|
| 2026-04-05 | interest-rate-risk | Convexity box, CFA exam tips, immunization warnings, further reading |
| 2026-04-05 | risk-return | **NEW sections:** MWR/TWR, nominal/real (Fisher), ERP, risk aversion & utility. Return types comparison table. Expanded all metric sections |
| 2026-04-05 | finite-difference | Heat equation analogy, CFL condition, Thomas algorithm, Rannacher smoothing, convergence guidelines |
| 2026-04-05 | sampling-estimation | CLT emphasis, CI misinterpretation correction, bootstrap principle, estimator properties, power analysis |
| 2026-04-05 | put-call-parity | Lower bound proof, arbitrage construction tips, American early exercise, dividend adjustments, synthetics table |
| 2026-04-05 | mean-variance | Markowitz insight, correlation-frontier shape, estimation challenges, two-fund vs one-fund, cost of constraints |
| 2026-04-05 | lattice-methods | Hand-worked 2-period tree, trinomial comparison table, node alignment, Hull-White calibration, Richardson extrapolation |
| 2026-04-05 | monte-carlo-pricing | Worked hand example, fundamental theorem, Ito correction, variance reduction deep dive, convergence tables |
| 2026-04-05 | geometric-brownian-motion | Donsker's theorem, history table, nowhere-differentiable paths, GBM solution derivation, drift vs vol estimation |
| 2026-04-05 | yield-curves | 4 term structure theories, hand-worked bootstrap, NS decomposition table, forward rate locking, inversion case study |
| 2026-04-05 | interest-rate-math | Rule of 72, EAR worked examples, origin of e, rate conversion roadmap, forward rate arbitrage, day count market table |
| 2026-04-05 | capm-factor-models | CAPM formula components, CML vs SML table, beta interpretation table, Blume adjustment, alpha testing, FF3 factors, active vs passive |
| 2026-04-05 | FSA/earnings-quality | 8 interpretation paragraphs, accrual ratio thresholds table, Jones model limitations, M-Score classification methodology, analyst workflow, CFA study guide |
| 2026-04-05 | FSA/intro-financial-statements | Cross-statement linkages, IS/BS/CFS reading guides, audit failures (Enron/WorldCom/Wirecard), IFRS→GAAP adjustment examples, key formulas table |
| 2026-04-05 | FSA/balance-sheet-working-capital | LIFO reserve trend analysis, depreciation tax shields, component depreciation, impairment worked example, goodwill, negative working capital, Dell CCC case, pension obligations |
| 2026-04-05 | FSA/income-statement-analysis | Revenue quality framework, gross margin benchmarks, operating leverage, R&D capitalisation IFRS/GAAP, adjusted EBITDA critique, effective tax rate diagnostics, EPS buyback distortion, Apple IS example, OCI worked example |

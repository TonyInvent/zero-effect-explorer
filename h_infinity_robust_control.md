# H-Infinity Robust Control: When You Know Your Model Is Wrong

**LQR gives you the optimal controller for the plant you told it about. H∞ gives you the best controller you can guarantee will work for the plant you actually have — which is never exactly the one in the model. If LQR is precision engineering, H∞ is engineering with a margin for error. And margins, in control, are the difference between a design that works on paper and one that works in the field.**

---

## 1. The problem LQR can't solve

Imagine you've designed an LQR controller for a motor. You measured the inertia, the resistance, the back-EMF constant. You built the $A$ and $B$ matrices. You solved the Riccati equation. The simulation is beautiful — fast rise time, zero overshoot, perfect rejection of step disturbances. You ship it.

Then the motor heats up. Resistance rises 30%. The load inertia doubles because the robot picked up a heavier part. The bearings wear in and friction changes. Suddenly your beautiful LQR oscillates. Or worse, it goes unstable.

What happened? **LQR optimizes for the nominal plant.** It finds the controller that minimizes a quadratic cost for *one specific* $G(s)$. If the real $G(s)$ is different, LQR makes no promises. It might still work. It might not. The LQR optimal cost $J$ is a guarantee — but only for the plant you modeled.

This is not a flaw in LQR. It's a flaw in the question LQR answers. LQR answers: "What's the best controller for this plant?" The question you should be asking is: **"What's the best controller that works for *every* plant in the uncertainty set?"**

H∞ control answers that question.

---

## 2. The H∞ mindset: design for the worst case

Let's make this concrete. You have a nominal model $G_0(s)$. You know the real plant $G(s)$ isn't exactly $G_0(s)$ — but you can bound how wrong the model might be. Maybe the gain at resonance is ±3 dB uncertain. Maybe a pole location is known only to within 10%. Maybe there's unmodeled high-frequency dynamics you deliberately neglected.

The set of all possible plants — the **uncertainty set** $\mathcal{G}$ — is the collection of all $G(s)$ that are "close enough" to $G_0(s)$ by some measure. You don't know which $G \in \mathcal{G}$ is the real one. You need a single controller $K(s)$ that:
1. **Stabilizes** every $G \in \mathcal{G}$ (robust stability)
2. **Performs adequately** for every $G \in \mathcal{G}$ (robust performance)

The H∞ approach is direct: define a measure of "how bad things can get" — the maximum closed-loop gain from disturbance to error, over all frequencies and over all plants in the uncertainty set — and then **minimize that worst-case value**.

$$ \text{Find } K \text{ that minimizes } \max_{\omega} \bar{\sigma}\big(T_{zw}(j\omega)\big) $$

Here $\bar{\sigma}$ is the maximum singular value (for MIMO) — it's the worst-case amplification at frequency $\omega$. The symbol $\|\cdot\|_\infty$ (pronounced "H-infinity norm") is exactly this: the peak magnitude across all frequencies. Minimizing $\|T_{zw}\|_\infty$ means minimizing the worst thing that can happen.

**A note on notation:** $T_{zw}$ means the closed-loop transfer function from $w$ to $z$, where:
- $w$ = **exogenous inputs** — everything that comes from outside the controller: reference commands, disturbances, sensor noise, anything that perturbs the system
- $z$ = **regulated outputs** — everything you want to keep small: tracking error, control effort, actuator rate, any signal whose size indicates poor performance

The subscripts $zw$ follow the standard control convention of "from (second subscript) → to (first subscript)." So $T_{zw}$ maps disturbances-and-commands $(w)$ to errors-and-effort $(z)$. This is formalized in §6 with the generalized plant $P(s)$, but the notation is used throughout because the fundamental H∞ problem is always "make the worst-case $w \to z$ gain as small as possible."

The contrast with LQR is stark:

| | LQR (H₂) | H∞ |
|---|---|---|
| **Optimizes** | Average (RMS) energy | Peak (worst-case) magnitude |
| **Plant assumption** | Exact nominal model | Nominal model + uncertainty bound |
| **Result** | Best nominal performance | Guaranteed robustness |
| **Guarantee** | None if model is wrong | Stability + performance for all $G \in \mathcal{G}$ |
| **Cost** | Higher nominal performance sacrificed for robustness |

H∞ is conservative by design. That's not a bug — it's the feature you're paying for.

---

## 3. Uncertainty: how to describe what you don't know

Before you can design for uncertainty, you need to describe it. How do you say "the plant is somewhere in this neighborhood" mathematically?

### 3.1 Additive uncertainty

The simplest: the real plant is the nominal plus some unknown perturbation:

$$G(s) = G_0(s) + \Delta(s) \cdot W_a(s)$$

where $\Delta(s)$ is any stable transfer function with $\|\Delta\|_\infty \leq 1$, and $W_a(s)$ is a frequency weight that tells you how big the uncertainty can be at each frequency. $W_a(s)$ is large at frequencies where you're very uncertain; small where you trust the model.

**When to use:** the uncertainty doesn't scale with the plant gain. An extra vibration mode, a sensor resonance, flexible-body dynamics you didn't model.

### 3.2 Multiplicative uncertainty

More common: the uncertainty is proportional to the plant's own response:

$$G(s) = G_0(s) \big(1 + \Delta(s) \cdot W_m(s)\big)$$

Now a 10% uncertainty means 10% of whatever the plant is doing at that frequency. This models parameter variations — resistance changes, gain drift, pole/zero migration.

**When to use:** parameter uncertainty. The plant structure is right; the numbers are wrong.

### 3.3 Coprime factor uncertainty

The most general and mathematically natural: perturb the numerator and denominator of a coprime factorization separately:

$$G = (M + \Delta_M)^{-1}(N + \Delta_N)$$

This captures uncertainty that moves *both* poles and zeros independently. It's the framework the gap metric uses to measure distance between plants, and it's what `ncfsyn` (normalized coprime factor H∞ synthesis) solves.

**When to use:** you don't know the model structure. Unmodeled dynamics, neglected coupling, flexible modes you didn't even know existed.

### 3.4 The uncertainty weight: encoding what you know

The weight $W(s)$ is where your engineering judgment enters. A typical weight:

$$W_m(s) = \frac{0.1 s + 1}{0.05 s + 1}$$

At DC ($\omega = 0$): $|W_m(0)| = 1$ — 100% uncertainty. You know the gain can vary a lot at steady state (friction, loading). At high frequency: $|W_m(j\infty)| = 2$ — 200% uncertainty. You're certain there are unmodeled dynamics up there. The crossover where $|W_m| = 1$ (at ~10 rad/s in this case) is the boundary between "mostly confident" and "mostly guessing."

Choosing $W(s)$ is not automatic. It requires plant knowledge. But the H∞ framework separates the hard part (how uncertain are you?) from the computation (what controller handles that uncertainty optimally?). You supply the uncertainty description; the algorithm supplies the controller.

---

## 4. The mixed-sensitivity problem: shaping three things at once

H∞ design is organized around three key closed-loop transfer functions. Each one measures a different aspect of what the controller is doing — and what it's failing at.

### 4.1 The sensitivity function $S(s)$

$$S(s) = \frac{1}{1 + G(s)K(s)}$$

$S$ is the transfer function from reference $r$ to error $e = r - y$. It's also the transfer function from output disturbance $d$ to output $y$. If you want the output to track the reference — and reject disturbances — you need $|S(j\omega)|$ to be small.

But $S$ can't be small everywhere. Bode's integral formula says: the area under $\log|S(j\omega)|$ for an unstable plant is fixed. Making $S$ small at some frequencies forces it to be large at others. This is the **waterbed effect** — you can push down the sensitivity in one frequency band, but it pops up somewhere else. H∞ design respects this trade-off automatically.

### 4.2 The complementary sensitivity $T(s)$

$$T(s) = \frac{G(s)K(s)}{1 + G(s)K(s)} = 1 - S(s)$$

$T$ is the transfer function from reference $r$ to output $y$ (when tracking), and from sensor noise $n$ to output $y$ (when rejecting noise). $T$ measures how faithfully the output follows the reference — and how much sensor noise passes through.

Robust stability is determined by $T$: if multiplicative uncertainty is bounded by $|W_m(j\omega)|$, the closed loop remains stable as long as:

$$|T(j\omega)| < \frac{1}{|W_m(j\omega)|} \quad \forall \omega$$

This is the **small-gain theorem** in action. $T$ must roll off before the uncertainty becomes significant — which is why every practical controller has limited bandwidth.

### 4.3 The control sensitivity $KS(s)$

$$KS(s) = \frac{K(s)}{1 + G(s)K(s)}$$

$KS$ is the transfer function from reference $r$ to control effort $u$. It measures how hard the controller is working. If $|KS(j\omega)|$ is large, small errors produce large control signals — the controller is "aggressive." Aggressive control can saturate actuators, excite unmodeled resonances, and waste energy.

### 4.4 The mixed-sensitivity formulation

Put these together with frequency weights, and the design problem becomes:

$$\left\| \begin{bmatrix} W_1(s) S(s) \\ W_2(s) KS(s) \\ W_3(s) T(s) \end{bmatrix} \right\|_\infty < 1$$

The weights mean:
- $W_1(s)$ shapes $S$ — performance (tracking, disturbance rejection)
- $W_2(s)$ shapes $KS$ — control effort (actuator limits, energy)
- $W_3(s)$ shapes $T$ — robustness (noise rejection, stability under uncertainty)

The goal: find $K(s)$ such that every row of this stacked transfer function has magnitude less than 1 at every frequency. If $\|W_1 S\|_\infty < 1$, then $|S(j\omega)| < 1/|W_1(j\omega)|$ — the sensitivity is shaped by the inverse of the weight.

---

## 5. Weighting functions: turning specs into math

The art of H∞ design is choosing $W_1$, $W_2$, $W_3$. Here is how you translate engineering specifications into transfer functions.

### 5.1 The performance weight $W_1(s)$

**Spec:** "Steady-state tracking error must be less than 0.1% for constant references. Above 100 rad/s, we don't care about tracking."

Translation: at DC ($s = 0$), $|S(0)| < 0.001$, so $|W_1(0)| > 1000$. The weight must be large at low frequency and small at high frequency — an integrator-like shape:

$$W_1(s) = \frac{s/M_s + \omega_B}{s + \omega_B \cdot A}$$

where:
- $A = 10^{-3}$ is the allowed steady-state error (0.1%) → DC gain of $1/A = 1000$
- $M_s = 2$ is the peak sensitivity allowed (prevents excessive amplification from the waterbed effect)
- $\omega_B = 100$ rad/s is the bandwidth requirement

At low frequencies: $|W_1| \approx 1/A$, large — forces $S$ small. At high frequencies: $|W_1| \approx 1/M_s$, small — allows $S$ to be large. The crossover at $\omega_B$ is where tracking stops mattering.

### 5.2 The control weight $W_2(s)$

**Spec:** "The actuator saturates at 10 N·m. Control effort above 500 rad/s is noise and shouldn't be amplified."

$$W_2(s) = \frac{s + \omega_c / M_u}{\varepsilon s + \omega_c}$$

where $\varepsilon$ limits low-frequency control effort (it grows large at low frequency, restricting $KS$), and $\omega_c$ sets where the weight rolls off. Above $\omega_c$, $W_2$ is small — the controller can amplify high frequencies (which you'll suppress with $W_3$ instead).

### 5.3 The robustness weight $W_3(s)$

**Spec:** "Unmodeled dynamics become significant above 300 rad/s. The complementary sensitivity must roll off before then."

If multiplicative uncertainty grows like $s$ (typical for unmodeled high-frequency dynamics), set:

$$W_3(s) = \frac{s + \omega_0 \cdot r_0}{r_\infty \cdot s + \omega_0 / r_\infty}$$

where $r_0 = 0.1$ (10% uncertainty at DC — parameter uncertainty), $r_\infty = 10$ (1000% at high frequency — unmodeled dynamics dominate), and $\omega_0 = 300$ rad/s (crossover where uncertainty transitions from "small" to "large").

At high frequency: $|W_3| \to r_\infty = 10$, so $1/|W_3| \to 0.1$ — forcing $|T| < 0.1$, a 20 dB roll-off above 300 rad/s.

### 5.4 The weighting matrix

The stacked transfer function becomes:

$$\begin{bmatrix} z_1 \\ z_2 \\ z_3 \end{bmatrix} = \begin{bmatrix} W_1 S \\ W_2 KS \\ W_3 T \end{bmatrix} \cdot w$$

The H∞ problem: find $K$ that makes $\|T_{zw}\|_\infty < 1$. If you can achieve this, all three specifications are simultaneously satisfied.

---

## 6. The standard H∞ problem

All H∞ design problems can be cast in a unified framework. This is not just mathematical formalism — it's what makes the solution computable.

### 6.1 The generalized plant

Arrange the plant, weights, and interconnection into a single "generalized plant" $P(s)$:

```
     ┌────────────────────────────┐
 w ─→│                            │─→ z
     │          P(s)              │
 u ─→│                            │─→ y
     └────────────────────────────┘
                ↑
                │
            ┌───────┐
            │ K(s)  │
            └───────┘
```

- $w$: exogenous inputs (references, disturbances, noise)
- $z$: regulated outputs (weighted error, weighted control, weighted output — the things you want to keep small)
- $u$: control inputs (actuator signals)
- $y$: measured outputs (sensor signals)

$P(s)$ partitions as:

$$\begin{bmatrix} z \\ y \end{bmatrix} = \begin{bmatrix} P_{11} & P_{12} \\ P_{21} & P_{22} \end{bmatrix} \begin{bmatrix} w \\ u \end{bmatrix}$$

The closed-loop transfer function from $w$ to $z$ with $u = Ky$ is the **lower linear fractional transformation** (LFT):

$$T_{zw} = \mathcal{F}_\ell(P, K) = P_{11} + P_{12} K (I - P_{22} K)^{-1} P_{21}$$

This is the map whose H∞ norm you want to minimize.

### 6.2 The H∞ optimal control problem

$$\text{Find a stabilizing } K(s) \text{ such that } \|\mathcal{F}_\ell(P, K)\|_\infty \text{ is minimized.}$$

Or, more commonly, the **suboptimal** problem:

$$\text{Find a stabilizing } K(s) \text{ such that } \|\mathcal{F}_\ell(P, K)\|_\infty < \gamma.$$

Solve for decreasing $\gamma$ (bisection) until no solution exists. The smallest achievable $\gamma$ is the optimal H∞ norm.

The output is a controller $K(s)$ that achieves the best possible worst-case performance for the weighted problem you specified.

---

## 7. The DGKF solution: two Riccati equations, one controller

The problem "find $K$ such that $\|T_{zw}\|_\infty < \gamma$" sounds intractable. In 1989, Doyle, Glover, Khargonekar, and Francis showed it reduces to checking two algebraic Riccati equations. This is what `hinfsyn` in MATLAB computes. Here's what it does.

### 7.1 State-space data

Start with a state-space realization of the generalized plant $P(s)$:

$$\dot{x} = A x + B_1 w + B_2 u$$
$$z = C_1 x + D_{11} w + D_{12} u$$
$$y = C_2 x + D_{21} w + D_{22} u$$

The solution requires some technical conditions: $(A, B_2)$ stabilizable, $(C_2, A)$ detectable, $D_{12}$ full column rank, $D_{21}$ full row rank. These ensure the problem is well-posed — there are no pole-zero cancellations on the imaginary axis.

### 7.2 The two Riccati equations

**Controller Riccati** ($X_\infty$, determines state feedback):

$$A^T X_\infty + X_\infty A + C_1^T C_1 + X_\infty (\gamma^{-2} B_1 B_1^T - B_2 B_2^T) X_\infty = 0$$

**Filter Riccati** ($Y_\infty$, determines output injection):

$$A Y_\infty + Y_\infty A^T + B_1 B_1^T + Y_\infty (\gamma^{-2} C_1^T C_1 - C_2^T C_2) Y_\infty = 0$$

These look like CARE equations with extra terms. The $\gamma^{-2}$ terms couple the two Riccati equations — they're not independent like in LQG. As $\gamma \to \infty$, the coupling disappears and you recover the separated LQG Riccati equations. As $\gamma$ decreases, the coupling strengthens until, at the optimal $\gamma$, one of the equations becomes singular.

### 7.3 The controller

If both Riccati solutions $X_\infty$ and $Y_\infty$ are positive semidefinite, and the spectral radius $\rho(X_\infty Y_\infty) < \gamma^2$, then a controller achieving $\|T_{zw}\|_\infty < \gamma$ exists:

$$\dot{\hat{x}} = A \hat{x} + B_2 u + Z_\infty L_\infty (y - C_2 \hat{x})$$
$$u = F_\infty \hat{x}$$

where:
- $F_\infty = -B_2^T X_\infty$ — the state feedback gain
- $L_\infty = -Y_\infty C_2^T$ — the output injection gain
- $Z_\infty = (I - \gamma^{-2} Y_\infty X_\infty)^{-1}$ — the coupling term

The controller is **observer-based**: estimate the state, then apply H∞-optimal state feedback. This is the same architecture as LQG — but the gains are different. They're chosen to minimize the *worst-case* response, not the *average*.

### 7.4 The bisection algorithm

`hinfsyn` doesn't solve one pair of Riccati equations. It solves dozens:

```
γ_high = large value (guaranteed feasible)
γ_low  = 0
while γ_high - γ_low > tolerance:
    γ = (γ_high + γ_low) / 2
    Solve controller Riccati for X∞
    Solve filter Riccati for Y∞
    if X∞ ≥ 0, Y∞ ≥ 0, and ρ(X∞ Y∞) < γ²:
        γ_high = γ   (feasible — try lower)
    else:
        γ_low = γ    (infeasible — must go higher)
return controller at γ_high
```

The output: a controller $K(s)$ whose order equals the order of the generalized plant $P(s)$ — which is the plant order *plus* all the weighting filter orders.

---

## 8. Why H∞ controllers are high-order (and what to do about it)

The generalized plant $P(s)$ has order:

$$\text{order}(P) = \text{order}(G) + \text{order}(W_1) + \text{order}(W_2) + \text{order}(W_3)$$

A 4th-order plant with three 2nd-order weights gives a 10th-order $P(s)$. The H∞ controller $K(s)$ inherits this full order. A 10th-order controller is mathematically optimal. It is also impractical — 10 states to maintain on a PLC running at 1 kHz.

This is the primary practical criticism of H∞ design. It's not that the theory is wrong — it's that the resulting controller is too complex to implement directly. But this is a solved problem.

### 8.1 Balanced truncation

Balanced truncation (Moore, 1981) reduces the controller order while preserving stability and providing an error bound.

The idea: transform the controller's state-space realization so that each state is equally "reachable" (easy to drive with the input) and "observable" (visible in the output). The Hankel singular values $\sigma_i$ measure how much each balanced state contributes to the input-output map. States with small $\sigma_i$ contribute almost nothing — you can truncate them.

The error bound: if you keep $r$ states out of $n$, the H∞ norm of the approximation error is:

$$\|K - K_r\|_\infty \leq 2 \sum_{i=r+1}^n \sigma_i$$

This bound is tight. You know exactly how much you're losing.

### 8.2 Typical reduction results

A 10th-order H∞ controller for a motor drive might have Hankel singular values: $\{4.2, 1.8, 0.31, 0.07, 0.002, \ldots\}$. Keeping 4 states gives an error bound of $2 \times 0.072 = 0.14$, which is negligible for most applications. The reduced 4th-order controller is implementable and nearly indistinguishable from the full-order optimum.

### 8.3 Alternative: directly constrain the order

Modern approaches solve the H∞ problem subject to a fixed controller order using non-smooth optimization (the `hinfstruct` command in MATLAB). Instead of designing high-order and reducing, you specify the controller order upfront and solve a non-convex but tractable optimization. The result is directly implementable.

---

## 9. H∞ through the Youla lens: what's really happening

The H∞ problem looks complex — generalized plants, Riccati equations, bisection. But the Youla parameterization reveals what's actually going on underneath.

Recall: the Youla parameterization says that **every** stabilizing controller can be written in terms of a free stable parameter $Q(s)$:

$$K(s) = \frac{U_0(s) + M(s)Q(s)}{V_0(s) - N(s)Q(s)}$$

And every closed-loop transfer function is **affine** in $Q$:

$$T_{zw}(Q) = T_0 + T_1 \cdot Q \cdot T_2$$

where $T_0, T_1, T_2$ are fixed (they depend only on $G$ and the anchor $K_0$).

The H∞ problem then becomes:

$$\text{Find stable } Q(s) \text{ that minimizes } \|T_0 + T_1 Q T_2\|_\infty$$

This is a **convex optimization problem** over a convex set (all stable $Q$). No local minima. No heuristics. The DGKF solution is just one way to solve this convex problem — it's an *interior-point method* in function space, producing the exact solution via Riccati equations.

This connection explains three things that are otherwise mysterious:

1. **Why H∞ works at all.** The problem is convex because every stabilizing controller lives in a convex set parameterized by $Q$. Zames realized this in 1981. Without Youla (1976), H∞ is a non-convex mess. With Youla, it's convex optimization.

2. **What the Riccati solution produces.** It finds the $Q$ that minimizes $\|T_{zw}(Q)\|_\infty$ — the point in $Q$-space that's closest (in H∞ norm) to the ideal closed-loop map. The resulting controller $K$ is the image of that optimal $Q$ under the Youla formula.

3. **Why the controller order explodes.** The optimal $Q$ is as complex as the problem demands. Since the closed-loop maps are affine in $Q$, and the weights add dynamics, the optimal $Q$ absorbs all that complexity. Youla guarantees the solution exists; it doesn't guarantee it's low-order.

In this light, H∞ is not a separate theory from LQR or LQG — it's the same convex problem (optimize over $Q$) with a different norm. LQG minimizes the H₂ norm; H∞ minimizes the H∞ norm. Both are convex over the Youla set. Both are solved by Riccati equations. The choice of norm encodes your attitude toward uncertainty: average it out (H₂) or prepare for the worst (H∞).

---

## 10. When to use H∞ (and when not to)

### Use H∞ when:

- **Model uncertainty is your primary concern.** Aerospace, flexible structures, chemical processes with poorly known parameters.
- **Frequency-domain specifications are given directly.** "±1% below 10 rad/s, ±10% above, roll off at 40 dB/decade after 100 rad/s" — this is exactly a weighting function specification.
- **You have competing frequency-domain requirements.** Tracking accuracy in one band, noise rejection in another, actuator limits in a third — mixed-sensitivity handles all three.
- **The plant has RHP zeros or delays.** H∞ explicitly accounts for fundamental limitations — it won't produce impossible demands (requesting high bandwidth beyond a RHP zero) the way unconstrained LQR can.
- **You need guarantees.** Certification (DO-178C, ISO 26262) sometimes requires proof of robust stability. H∞ + μ-analysis provides that.

### Don't use H∞ when:

- **Your model is accurate and uncertainty is low.** LQR/LQG will give better nominal performance.
- **You have hard input/state constraints.** H∞ handles frequency-domain specs; MPC handles time-domain constraints (saturation, rate limits). For constrained problems, use MPC.
- **You need a low-order controller on a resource-constrained platform.** You can reduce the order afterward, but the design workflow is heavier than PID or LQR.
- **You're tuning interactively.** H∞ design requires specifying weighting functions — you can't just turn a knob and see what happens the way you can with PID gains or LQR's Q/R matrices. (Though the interactive explorer tools in this project help bridge that gap.)

### The practical middle ground

In industry, H∞ is often used as a **design tool**, not a final implementation. The workflow:

1. Design an H∞ controller with appropriate weights
2. Reduce its order via balanced truncation
3. Use the reduced controller as a reference — tune a PID or low-order structure to match its frequency response

You get the benefits of systematic robust design without the implementation complexity. This is routine in aerospace flight control: H∞ design → model reduction → gain-scheduled implementation.

---

## 11. Connection to this project

| Doc | The H∞ connection |
|-----|-------------------|
| `youla_parameterization.md` | Youla makes the set of stabilizing controllers convex. H∞ minimizes a convex objective (the H∞ norm) over this convex set. Without Youla, H∞ is non-convex and intractable. With Youla, it's solved by Riccati equations. Section 9 of this doc explains this in detail. |
| `core_problems_controller_design.md` | Problem #4 (model uncertainty) is exactly what H∞ addresses. Problem #9 (multi-objective trade-offs) — speed vs robustness, tracking vs noise — is what the mixed-sensitivity weights formalize. |
| `care_vs_dare.md` | The DGKF solution in Section 7 uses two coupled Riccati equations. As $\gamma \to \infty$, they decouple into the standard CARE equations of LQG. H∞ is LQG with a constraint on the worst-case gain. |
| `lead_lag_compensator_design.md` | Lead-lag design shapes the open-loop Bode plot manually. H∞ does the same thing automatically, given weighting functions. The weights $W_1, W_3$ encode the same intent as lead (add phase, increase bandwidth) and lag (boost DC gain). |
| `lqr_explorer.html` | LQR optimizes the H₂ norm (RMS energy) over the Youla set. H∞ optimizes the H∞ norm (peak magnitude) over the same set. The explorer shows how Q/R affect closed-loop poles; H∞ design shows how weights affect the frequency response directly. |
| `pid_explorer.html` | PID gains are a 3-parameter restriction of the full $Q$-space. H∞ can design a full-order controller, then you can project it onto the PID subspace. The result is a PID with H∞-optimal gains — systematically tuned, not hand-tweaked. |
| `waterbed_effect.md` | Bode's integral constraint: making $S$ small at some frequencies forces it large at others. H∞ weighting functions respect this fundamental limit — $W_1$ can't demand performance that violates the integral. The $\|W_1 S\|_\infty < 1$ constraint automatically enforces the waterbed trade-off. |

---

## 12. Further reading

**Start here — the most intuitive:**
- Skogestad, S. & Postlethwaite, I. (2005). *Multivariable Feedback Control: Analysis and Design*, 2nd ed. Wiley. Chapters 2, 7–9. The gold standard for learning H∞. Chapter 2 on frequency-domain specifications, Chapters 7–8 on $S$/$T$ shaping and weights, Chapter 9 on the full H∞ solution. Written for engineers, not mathematicians.

**The DGKF solution in detail:**
- Doyle, J.C., Glover, K., Khargonekar, P.P., & Francis, B.A. (1989). "State-space solutions to standard H₂ and H∞ control problems." *IEEE Trans. Automatic Control*, 34(8), 831–847. The paper that made H∞ computable. Dense but essential — if you want to understand what `hinfsyn` actually does.

**The historical chain:**
- Zames, G. (1981). "Feedback and optimal sensitivity: Model reference transformations, multiplicative seminorms, and approximate inverses." *IEEE Trans. Automatic Control*, 26(2), 301–320. The paper that connected Youla to H∞ and launched robust control. Introduced the H∞ norm as the right measure for robustness.
- Youla, D.C., Jabr, H.A., Bongiorno, J.J. (1976). "Modern Wiener-Hopf design of optimal controllers — Part II." *IEEE Trans. Automatic Control*, 21(3), 319–338. The Youla parameterization — the structural result that makes H∞ convex.

**The comprehensive reference:**
- Zhou, K., Doyle, J.C., & Glover, K. (1996). *Robust and Optimal Control*. Prentice-Hall. Chapters 14–18. Everything: small-gain theorem, uncertainty modeling, mixed sensitivity, DGKF, μ-analysis, model reduction. The definitive technical reference.

**Model reduction:**
- Moore, B.C. (1981). "Principal component analysis in linear systems: Controllability, observability, and model reduction." *IEEE Trans. Automatic Control*, 26(1), 17–32. Balanced truncation — the standard method for reducing high-order H∞ controllers.
- Antoulas, A.C. (2005). *Approximation of Large-Scale Dynamical Systems*. SIAM. The modern treatment of model reduction, including balanced truncation, Hankel norm approximation, and Krylov methods.

**The free classic:**
- Doyle, J.C., Francis, B.A., & Tannenbaum, A.R. (1992). *Feedback Control Theory*. Macmillan. Available as a free PDF. Chapter 8 on H∞ and Chapter 9 on design constraints. Written for undergraduates — the clearest exposition of why H∞ exists and what it solves.

**MATLAB implementations to study:**
- `hinfsyn` — the standard H∞ solver based on DGKF
- `mixsyn` — mixed-sensitivity H∞ design (a wrapper that builds $P(s)$ from weights and calls `hinfsyn`)
- `ncfsyn` — loop-shaping H∞ design using normalized coprime factors (often more intuitive: shape the open loop, then robustly stabilize)
- `hinfstruct` — fixed-structure H∞ synthesis (optimize a PID's gains subject to H∞ constraints)
- `balred` / `reduce` — balanced truncation and model reduction

---
hide:
  - navigation
  - toc
---

<div class="project-home">
  <header class="project-intro">
    <div class="project-heading">
      <img class="project-logo" src="assets/fr3-stack.jpg" alt="" width="128" height="128">
      <div class="project-brand">
        <p class="project-byline">DexLab</p>
        <h1>fr3-stack</h1>
        <p class="project-description">Control suite for the <br>Franka Research 3.</p>
      </div>
    </div>
    <div class="project-summary">
      <p>A Python client on your workstation. A real-time C++ controller on the NUC. Connected through libfranka, without ROS.</p>
      <p>Cartesian impedance, hybrid force/position control, and software-level dual-arm coordination.</p>
      <div class="project-actions"><a class="project-primary" href="quickstart.html">Get started <span aria-hidden="true">→</span></a><a href="https://github.com/Robot-Dexterity-Lab/fr3_stack">GitHub <span aria-hidden="true">↗</span></a></div>
    </div>
  </header>
  <div class="project-resources">
    <section class="project-start">
      <h2>Start with a connection</h2>
      <p>Set up the NUC daemon and install the Python client using the <a href="quickstart.html">installation guide</a>. Then read state from your workstation:</p>
      <div class="project-code"><span class="code-label">Python · Read robot state</span><pre><code><span class="code-keyword">from</span> fr3_stack <span class="code-keyword">import</span> Robot

<span class="code-keyword">with</span> Robot(<span class="code-string">"192.168.1.8"</span>) <span class="code-keyword">as</span> robot:
    state = robot.wait_for_state(timeout=5.0)
    print(state.pos, state.quat_xyzw)</code></pre></div>
      <p class="project-note">Connect to the NUC's IP address. This example reads state only.</p>
    </section>
    <section class="project-guides">
      <h2>Explore the stack</h2>
      <a class="guide-row" href="single-arm.html"><span><strong>Single-arm control</strong><small>State, poses, profiles, and Python interfaces</small></span><span aria-hidden="true">→</span></a>
      <a class="guide-row" href="controllers.html"><span><strong>Controllers</strong><small>Impedance, admittance, and force/position control</small></span><span aria-hidden="true">→</span></a>
      <a class="guide-row" href="dual-arm.html"><span><strong>Dual-arm coordination</strong><small>Two NUCs, paired targets, and fault handling</small></span><span aria-hidden="true">→</span></a>
      <a class="guide-row" href="troubleshooting.html"><span><strong>Troubleshooting</strong><small>Connections, F/T data, and build issues</small></span><span aria-hidden="true">→</span></a>
    </section>
  </div>
  <footer class="project-bottom">
    <p><strong>Development status</strong> — Dual-arm coordination provides paired dispatch and fault handling. NUC execution synchronization and real-robot evaluation are pending. <a href="dual-arm-coordination-plan.html">Roadmap →</a></p>
    <p><a href="development.html">Contributing</a> · <a href="https://github.com/Robot-Dexterity-Lab/fr3_stack/blob/main/AGENTS.md">Agent guide</a> · <a href="https://github.com/Robot-Dexterity-Lab/fr3_stack/issues">Report an issue</a></p>
  </footer>
</div>

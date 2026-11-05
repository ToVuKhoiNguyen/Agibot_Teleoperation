
function onXRFrame(time, frame) {
  if (!xrSession) return;
  xrSession.requestAnimationFrame(onXRFrame);

  // Render scene through WebXR camera
  renderer.render(scene, camera);

  if (!xrRefSpace) return;

  // Extract controller poses and buttons
  const inputSources = xrSession.inputSources || [];
  let leftPacket = null;
  let rightPacket = null;

  for (let i = 0; i < inputSources.length; i++) {
    const src = inputSources[i];
    if (!src || !src.gamepad) continue;
    const handedness = src.handedness;
    if (handedness !== 'left' && handedness !== 'right') continue;

    // Pose: targetRaySpace or gripSpace
    const space = src.gripSpace || src.targetRaySpace;
    let posArr = null;
    let quatArr = null;
    if (space) {
      const pose = frame.getPose(space, xrRefSpace);
      if (pose && pose.transform) {
        const p = pose.transform.position;
        const q = pose.transform.orientation;
        posArr = [p.x, p.y, p.z];
        // WebXR (x, y, z, w) -> Python (w, x, y, z)
        quatArr = [q.w, q.x, q.y, q.z];
      }
    }

    const gp = src.gamepad;
    const btns = gp.buttons || [];
    const axes = gp.axes || [0.0, 0.0];

    // Trigger = Button 0
    let triggerVal = 0.0;
    if (btns.length > 0) {
      triggerVal = typeof btns[0].value === 'number' ? btns[0].value : (btns[0].pressed ? 1.0 : 0.0);
    }

    // Grip = Button 1
    let gripVal = 0.0;
    let gripPressed = false;
    if (btns.length > 1) {
      gripVal = typeof btns[1].value === 'number' ? btns[1].value : (btns[1].pressed ? 1.0 : 0.0);
      gripPressed = gripVal >= 0.25 || Boolean(btns[1].pressed);
    }

    // Thumbstick click = Button 3
    let thumbClick = false;
    if (btns.length > 3) {
      thumbClick = Boolean(btns[3].pressed || (typeof btns[3].value === 'number' && btns[3].value > 0.5));
    }

    // Primary button (A on right, X on left) = Button 4
    let btnPrimary = false;
    if (btns.length > 4) {
      btnPrimary = Boolean(btns[4].pressed || (typeof btns[4].value === 'number' && btns[4].value > 0.5));
    }

    // Secondary button (B on right, Y on left) = Button 5
    let btnSecondary = false;
    if (btns.length > 5) {
      btnSecondary = Boolean(btns[5].pressed || (typeof btns[5].value === 'number' && btns[5].value > 0.5));
    }

    // Menu button = Button 6 (or reserved by system)
    let btnMenu = false;
    if (btns.length > 6) {
      btnMenu = Boolean(btns[6].pressed || (typeof btns[6].value === 'number' && btns[6].value > 0.5));
    }

    // Thumbstick axes
    let stickX = axes.length > 0 ? axes[0] : 0.0;
    let stickY = axes.length > 1 ? axes[1] : 0.0;
    if (Math.abs(stickX) < 0.05 && Math.abs(stickY) < 0.05 && axes.length >= 4) {
      if (Math.abs(axes[2]) > 0.05 || Math.abs(axes[3]) > 0.05) {
        stickX = axes[2];
        stickY = axes[3];
      }
    }

    const handData = {
      pos: posArr,
      quat: quatArr,
      grip: gripPressed,
      grip_val: gripVal,
      trigger: triggerVal,
      thumbstick: [stickX, stickY],
      thumbstick_click: thumbClick,
      btn_menu: btnMenu,
      buttons: {
        primary: btnPrimary,   // X on Left, A on Right
        secondary: btnSecondary, // Y on Left, B on Right
        menu: btnMenu
      }
    };
    if (handedness === 'left') {
      handData.btn_x = btnPrimary;
      handData.btn_y = btnSecondary;
      leftPacket = handData;
    } else {
      handData.btn_a = btnPrimary;
      handData.btn_b = btnSecondary;
      rightPacket = handData;
    }
  }

  // Send over /ws_panel at display rate
  if (leftPacket || rightPacket) {
    sendCmd({
      cmd: 'xr_controllers',
      t: performance.now() / 1000.0,
      rotation_enabled: xrRotationEnabled,
      left: leftPacket,
      right: rightPacket
    });
  }
}


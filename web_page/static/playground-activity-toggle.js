function createHandledActivityError(message) {
  const error = new Error(message);
  error.handled = true;
  return error;
}

async function stopActivitySession() {
  try {
    await fetch("/stop_data", { method: "POST" });
  } catch (error) {
    console.error("Unable to stop activity session.", error);
  }
}

function createActivityToggle(options) {
  const button =
    typeof options.buttonId === "string"
      ? document.getElementById(options.buttonId)
      : options.button;

  if (!button) {
    throw new Error("Activity toggle button was not found.");
  }

  const idleLabel = options.idleLabel || button.textContent.trim();
  const activeLabel = options.activeLabel || idleLabel;

  let isActive = false;
  let startController = null;

  function render() {
    button.classList.toggle("is-active", isActive);
    button.textContent = isActive ? activeLabel : idleLabel;
  }

  function setActive(nextValue) {
    isActive = Boolean(nextValue);
    if (!isActive) {
      startController = null;
    }
    render();
  }

  async function activate() {
    startController = new AbortController();
    setActive(true);

    try {
      await options.start({
        signal: startController.signal,
      });
    } catch (error) {
      if (error && error.name === "AbortError") {
        return;
      }

      if (typeof options.onError === "function") {
        options.onError(error);
      } else {
        console.error(error);
      }
    }
  }

  async function deactivate() {
    if (startController) {
      startController.abort();
      startController = null;
    }

    if (typeof options.stop === "function") {
      await options.stop();
    }

    setActive(false);

    if (typeof options.onReset === "function") {
      options.onReset();
    }
  }

  button.addEventListener("click", () => {
    if (isActive) {
      deactivate();
      return;
    }

    activate();
  });

  render();

  return {
    isActive: () => isActive,
    setActive,
    deactivate,
  };
}

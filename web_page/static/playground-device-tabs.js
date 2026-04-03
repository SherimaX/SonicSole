function updateGroupSummary(panel) {
  const summary = panel.querySelector("[data-group-summary]");
  if (!summary) return;

  const activeButtons = Array.from(
    panel.querySelectorAll(".pg-device-tab.is-active")
  );

  const labels = activeButtons.map((button) => button.textContent.trim());
  const prefix = labels.length === 1 ? "Viewing group: " : "Viewing groups: ";
  summary.textContent = prefix + labels.join(", ");
}

document.querySelectorAll(".pg-group-panel").forEach((panel) => {
  panel.querySelectorAll(".pg-device-tab").forEach((button) => {
    button.setAttribute(
      "aria-pressed",
      button.classList.contains("is-active") ? "true" : "false"
    );
  });

  updateGroupSummary(panel);
});

document.addEventListener("click", function (event) {
  const tab = event.target.closest(".pg-device-tab");
  if (!tab) return;

  const panel = tab.closest(".pg-group-panel");
  if (!panel) return;

  const buttons = Array.from(panel.querySelectorAll(".pg-device-tab"));
  const activeButtons = buttons.filter((button) =>
    button.classList.contains("is-active")
  );
  const isActive = tab.classList.contains("is-active");

  if (isActive && activeButtons.length === 1) {
    return;
  }

  if (!isActive && activeButtons.length >= 5) {
    return;
  }

  tab.classList.toggle("is-active");
  tab.setAttribute(
    "aria-pressed",
    tab.classList.contains("is-active") ? "true" : "false"
  );

  updateGroupSummary(panel);
});

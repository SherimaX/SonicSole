(function () {
  let currentLink = document.querySelector("link[data-live-css-src]");
  const sourceUrl = currentLink?.dataset.liveCssSrc;

  let lastModified = null;
  let updateInFlight = false;
  let viewportUpdateHandle = null;

  function versionedUrl(url) {
    const nextUrl = new URL(url, window.location.origin);
    nextUrl.searchParams.set("_cssLive", Date.now().toString());
    return nextUrl.toString();
  }

  async function checkForUpdates() {
    if (document.visibilityState === "hidden" || updateInFlight) return;
    if (!sourceUrl || !currentLink) return;

    try {
      const response = await fetch(sourceUrl, {
        method: "HEAD",
        cache: "no-store",
      });

      if (!response.ok) return;

      const nextModified = response.headers.get("Last-Modified");
      if (!nextModified) return;

      if (lastModified === null) {
        lastModified = nextModified;
        return;
      }

      if (nextModified === lastModified) return;

      updateInFlight = true;
      lastModified = nextModified;

      const replacement = currentLink.cloneNode(true);
      replacement.href = versionedUrl(sourceUrl);
      replacement.addEventListener(
        "load",
        function () {
          currentLink.remove();
          currentLink = replacement;
        },
        { once: true }
      );

      currentLink.parentNode.insertBefore(replacement, currentLink.nextSibling);
    } catch (error) {
      // Ignore transient polling issues in development.
    } finally {
      updateInFlight = false;
    }
  }

  function resolveDisplayMode() {
    if (window.navigator.standalone === true) {
      return {
        mode: "standalone",
        installed: true,
        source: "navigator.standalone",
      };
    }

    if (!window.matchMedia) {
      return {
        mode: "browser",
        installed: false,
        source: "none",
      };
    }

    const matchedQuery = [
      ["(display-mode: standalone)", "standalone"],
      ["(display-mode: fullscreen)", "fullscreen"],
      ["(display-mode: minimal-ui)", "minimal-ui"],
    ].find(function (entry) {
      return window.matchMedia(entry[0]).matches;
    });

    if (matchedQuery) {
      return {
        mode: matchedQuery[1],
        installed: true,
        source: matchedQuery[0],
      };
    }

    return {
      mode: "browser",
      installed: false,
      source: "none",
    };
  }

  function isAppleMobileDevice() {
    const userAgent = window.navigator.userAgent || "";
    return /iPhone|iPad|iPod/i.test(userAgent)
      || (window.navigator.platform === "MacIntel" && window.navigator.maxTouchPoints > 1);
  }

  function updateScrollableShellState() {
    [
      ".pg-phone-home",
      ".setup-guide-shell",
      ".pg-activity-stage",
    ].forEach(function (selector) {
      document.querySelectorAll(selector).forEach(function (element) {
        const isOverflowing = element.scrollHeight > element.clientHeight + 2;
        element.classList.toggle("pg-overflows-y", isOverflowing);
      });
    });
  }

  function updateViewportInsets() {
    const root = document.documentElement;
    const body = document.body;
    if (!root || !body) return;

    const displayState = resolveDisplayMode();
    const isStandalone = displayState.installed;
    const isAppleMobile = isAppleMobileDevice();
    const visualViewport = window.visualViewport;
    const supportsDynamicViewport = Boolean(
      window.CSS
      && typeof window.CSS.supports === "function"
      && window.CSS.supports("height: 100dvh")
    );
    const preferredViewportHeight = isAppleMobile && isStandalone
      ? "100vh"
      : supportsDynamicViewport
        ? "100dvh"
        : "100vh";

    body.classList.toggle("pg-standalone-webapp", isStandalone);
    body.classList.toggle("pg-ios-browser-web", isAppleMobile && !isStandalone);
    body.classList.toggle("pg-display-browser", displayState.mode === "browser");
    body.classList.toggle("pg-display-standalone", displayState.mode === "standalone");
    body.classList.toggle("pg-display-fullscreen", displayState.mode === "fullscreen");
    body.classList.toggle("pg-display-minimal-ui", displayState.mode === "minimal-ui");
    body.dataset.displayMode = displayState.mode;
    body.dataset.displaySource = displayState.source;
    body.dataset.displayInstalled = String(displayState.installed);

    let topOffset = 0;
    let bottomOffset = 0;

    if (isAppleMobile && !isStandalone && visualViewport) {
      topOffset = Math.max(0, Math.round(visualViewport.offsetTop));
      bottomOffset = Math.max(
        0,
        Math.round(window.innerHeight - visualViewport.height - visualViewport.offsetTop)
      );
    }

    root.style.setProperty("--pg-phone-viewport-height", preferredViewportHeight);
    root.style.setProperty("--pg-browser-top-offset", `${topOffset}px`);
    root.style.setProperty("--pg-browser-bottom-offset", `${bottomOffset}px`);
    updateScrollableShellState();
  }

  function scheduleViewportUpdate() {
    if (viewportUpdateHandle !== null) return;

    viewportUpdateHandle = window.requestAnimationFrame(function () {
      viewportUpdateHandle = null;
      updateViewportInsets();
    });
  }

  if (sourceUrl && currentLink) {
    window.setInterval(checkForUpdates, 1500);
  }

  if (document.readyState === "loading") {
    document.addEventListener("DOMContentLoaded", scheduleViewportUpdate, { once: true });
  } else {
    scheduleViewportUpdate();
  }

  window.addEventListener("resize", scheduleViewportUpdate);
  window.addEventListener("orientationchange", scheduleViewportUpdate);
  document.addEventListener("visibilitychange", scheduleViewportUpdate);

  if (window.visualViewport) {
    window.visualViewport.addEventListener("resize", scheduleViewportUpdate);
    window.visualViewport.addEventListener("scroll", scheduleViewportUpdate);
  }

  if (window.matchMedia) {
    [
      "(display-mode: standalone)",
      "(display-mode: fullscreen)",
      "(display-mode: minimal-ui)",
    ].forEach(function (query) {
      const media = window.matchMedia(query);
      if (typeof media.addEventListener === "function") {
        media.addEventListener("change", scheduleViewportUpdate);
      } else if (typeof media.addListener === "function") {
        media.addListener(scheduleViewportUpdate);
      }
    });
  }
})();

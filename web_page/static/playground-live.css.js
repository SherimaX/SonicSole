(function () {
  let currentLink = document.querySelector("link[data-live-css-src]");
  if (!currentLink) return;

  const sourceUrl = currentLink.dataset.liveCssSrc;
  if (!sourceUrl) return;

  let lastModified = null;
  let updateInFlight = false;

  function versionedUrl(url) {
    const nextUrl = new URL(url, window.location.origin);
    nextUrl.searchParams.set("_cssLive", Date.now().toString());
    return nextUrl.toString();
  }

  async function checkForUpdates() {
    if (document.visibilityState === "hidden" || updateInFlight) return;

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

  window.setInterval(checkForUpdates, 1500);
})();

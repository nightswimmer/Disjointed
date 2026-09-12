/* Disjointed manual - page behaviour.
   - Theme: taken from ?theme= on load, then from postMessage {type:"theme"} sent by the
     app's help drawer (src/help.ts) whenever the app's theme changes.
   - Topics: the URL hash (or postMessage {type:"goto"}) selects a <section class="topic">,
     highlights it and scrolls it into view. Unknown topics land on the table of contents.
   - Table of contents: generated from the chapter headings and topic sections.
   - Figures: <figure data-svg="img/x.svg"> is fetched and inlined (so the SVG's CSS
     variables follow the theme); <img data-dark="..." data-light="..."> swaps per theme. */
(function () {
  "use strict";

  var root = document.documentElement;
  var theme = "dark";

  function setTheme(t) {
    theme = t === "light" ? "light" : "dark";
    root.setAttribute("data-theme", theme);
    var imgs = document.querySelectorAll("img[data-dark][data-light]");
    for (var i = 0; i < imgs.length; i++) {
      var img = imgs[i];
      var src = img.getAttribute(theme === "light" ? "data-light" : "data-dark");
      if (img.getAttribute("src") !== src) img.setAttribute("src", src);
    }
  }

  var params = new URLSearchParams(location.search);
  setTheme(params.get("theme") || "dark");

  // --- table of contents -----------------------------------------------------------
  function buildToc() {
    var list = document.getElementById("toc-list");
    if (!list) return;
    var chapters = document.querySelectorAll("h1.chapter");
    for (var i = 0; i < chapters.length; i++) {
      var h = chapters[i];
      var li = document.createElement("li");
      var title = document.createElement("div");
      title.className = "toc-chapter";
      title.textContent = h.textContent;
      li.appendChild(title);
      var ul = document.createElement("ul");
      for (var n = h.nextElementSibling; n && !(n.tagName === "H1"); n = n.nextElementSibling) {
        if (!n.classList.contains("topic") || n.id === "toc") continue;
        var h2 = n.querySelector("h2");
        if (!h2) continue;
        var item = document.createElement("li");
        var a = document.createElement("a");
        a.href = "#" + n.id;
        a.textContent = (h2.childNodes[0] && h2.childNodes[0].textContent || h2.textContent).trim();
        item.appendChild(a);
        ul.appendChild(item);
      }
      li.appendChild(ul);
      list.appendChild(li);
    }
  }

  // --- topics ----------------------------------------------------------------------------
  var current = null;
  function goto(topic, smooth) {
    var id = (topic || "").replace(/^#/, "");
    var el = id && document.getElementById(id);
    if (!el || !el.classList.contains("topic")) el = document.getElementById("toc");
    if (!el) return;
    if (current) current.classList.remove("current");
    current = el;
    if (el.id !== "toc") el.classList.add("current");
    if (location.hash !== "#" + el.id) history.replaceState(null, "", "#" + el.id);
    el.scrollIntoView({ block: "start", behavior: smooth ? "smooth" : "auto" });
  }
  window.addEventListener("hashchange", function () {
    goto(location.hash, true);
  });
  window.addEventListener("message", function (e) {
    var d = e.data;
    if (!d || typeof d !== "object") return;
    if (d.type === "theme") setTheme(d.theme);
    else if (d.type === "goto") goto(d.topic, true);
  });

  // --- figures ---------------------------------------------------------------------------
  function inlineSvg(fig) {
    var url = fig.getAttribute("data-svg");
    if (!url || fig.getAttribute("data-loaded")) return;
    fig.setAttribute("data-loaded", "1");
    fetch(url)
      .then(function (r) {
        if (!r.ok) throw new Error(r.status + " " + r.statusText);
        return r.text();
      })
      .then(function (text) {
        var doc = new DOMParser().parseFromString(text, "image/svg+xml");
        var svg = doc.documentElement;
        if (!svg || svg.nodeName.toLowerCase() !== "svg") throw new Error("not an SVG");
        var cap = fig.querySelector("figcaption");
        fig.classList.remove("pending");
        fig.insertBefore(document.importNode(svg, true), cap);
        // The width/height attributes are the capture's CSS size; let it shrink to fit.
        var w = svg.getAttribute("width");
        if (w) fig.firstElementChild.style.maxWidth = w + "px";
        fig.firstElementChild.removeAttribute("height");
        fig.firstElementChild.style.width = "100%";
      })
      .catch(function (err) {
        fig.classList.add("pending");
        var cap = fig.querySelector("figcaption");
        var msg = document.createElement("div");
        msg.textContent = "Illustration not available (" + err.message + ")";
        fig.insertBefore(msg, cap);
      });
  }
  var figs = document.querySelectorAll("figure[data-svg]");
  if ("IntersectionObserver" in window) {
    var io = new IntersectionObserver(function (entries) {
      entries.forEach(function (en) {
        if (en.isIntersecting) {
          inlineSvg(en.target);
          io.unobserve(en.target);
        }
      });
    }, { rootMargin: "400px" });
    for (var i = 0; i < figs.length; i++) {
      figs[i].classList.add("pending");
      io.observe(figs[i]);
    }
  } else {
    for (var j = 0; j < figs.length; j++) inlineSvg(figs[j]);
  }

  // --- toolbar glyphs in headings --------------------------------------------------------
  // glyphs.js (generated from index.html by the manual generator) maps tool-x / mode-x /
  // element ids to the buttons' SVG markup. A section shows the buttons that lead to it:
  // by default those the app maps to its topic, or the keys listed in data-glyph.
  function addGlyphs() {
    var G = window.DISJOINTED_GLYPHS;
    if (!G) return;
    var M = window.DISJOINTED_TOPIC_IDS || {};
    var sections = document.querySelectorAll("section.topic");
    for (var i = 0; i < sections.length; i++) {
      var sec = sections[i];
      if (sec.id === "toc") continue;
      var h2 = sec.querySelector("h2");
      if (!h2) continue;
      var keys = sec.getAttribute("data-glyph");
      keys = keys ? keys.split(/\s+/) : (G[sec.id] ? [sec.id] : (M[sec.id] || []));
      var holder = null;
      for (var k = 0; k < keys.length; k++) {
        var svgs = G[keys[k]];
        if (!svgs) continue;
        if (!holder) {
          holder = document.createElement("span");
          holder.className = "glyphs";
        }
        for (var s = 0; s < svgs.length; s++) {
          var b = document.createElement("span");
          b.className = "glyph btn";
          b.innerHTML = svgs[s];
          holder.appendChild(b);
        }
      }
      if (holder) h2.insertBefore(holder, h2.firstChild);
    }
  }

  // Toolbar rows: <figure data-glyph-row="key key group:id ..."> draws the listed buttons
  // (a `group:` token expands to every button of that toolbar group) as a row of glyphs.
  function addGlyphRows() {
    var G = window.DISJOINTED_GLYPHS;
    var GR = window.DISJOINTED_GROUPS || {};
    if (!G) return;
    var figs = document.querySelectorAll("figure[data-glyph-row]");
    for (var i = 0; i < figs.length; i++) {
      var fig = figs[i];
      var keys = [];
      fig.getAttribute("data-glyph-row").split(/\s+/).forEach(function (t) {
        if (t.indexOf("group:") === 0) keys = keys.concat(GR[t.slice(6)] || []);
        else if (t) keys.push(t);
      });
      var row = document.createElement("div");
      row.className = "glyph-row";
      for (var k = 0; k < keys.length; k++) {
        var svgs = G[keys[k]];
        if (!svgs) continue;
        var b = document.createElement("span");
        b.className = "glyph btn";
        b.innerHTML = svgs[0]; // a button with several glyphs shows one at a time
        row.appendChild(b);
      }
      fig.insertBefore(row, fig.querySelector("figcaption"));
    }
  }

  addGlyphs();
  addGlyphRows();
  buildToc();
  goto(location.hash, false);
})();

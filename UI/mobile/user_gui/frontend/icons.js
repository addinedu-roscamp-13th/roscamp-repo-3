
  var ICONS={
    menu:'<svg viewBox="0 0 24 24"><path d="M4 7h16M4 12h16M4 17h16"/></svg>',
    robot:'<svg viewBox="0 0 24 24"><rect x="4" y="8" width="16" height="11" rx="2"/><path d="M12 8V4M9 4h6"/><circle cx="9" cy="13" r="1.3"/><circle cx="15" cy="13" r="1.3"/></svg>',
    battery:'<svg viewBox="0 0 24 24"><rect x="3" y="8" width="16" height="9" rx="2"/><path d="M21 11v3"/><rect x="5.5" y="10.5" width="8" height="4" rx="1" fill="currentColor" stroke="none"/></svg>',
    bell:'<svg viewBox="0 0 24 24"><path d="M6 16v-5a6 6 0 1 1 12 0v5l1.5 2h-15z"/><path d="M10 20a2 2 0 0 0 4 0"/></svg>',
    stop:'<svg viewBox="0 0 24 24"><path d="M8 3h8l5 5v8l-5 5H8l-5-5V8z"/><path d="M9.5 9.5l5 5M14.5 9.5l-5 5"/></svg>',
    person:'<svg viewBox="0 0 24 24"><circle cx="12" cy="8" r="4"/><path d="M4.5 20c1-4 4.5-6 7.5-6s6.5 2 7.5 6"/></svg>',
    shield:'<svg viewBox="0 0 24 24"><path d="M12 3l7 3v5c0 5-3.5 8-7 9-3.5-1-7-4-7-9V6z"/><path d="M9 12l2 2 4-4"/></svg>',
    call:'<svg viewBox="0 0 24 24"><path d="M3 10v4l9 4V6z"/><path d="M14 8l6-3v14l-6-3"/></svg>',
    gift:'<svg viewBox="0 0 24 24"><rect x="4" y="9" width="16" height="11" rx="1"/><path d="M3 9h18M12 9v11"/><path d="M12 9c-1-3-5-4-5-1s4 1 5 1zM12 9c1-3 5-4 5-1s-4 1-5 1z"/></svg>',
    motion:'<svg viewBox="0 0 24 24"><polyline points="3 12 7 12 10 4 14 20 17 12 21 12"/></svg>',
    pick:'<svg viewBox="0 0 24 24"><circle cx="12" cy="8.5" r="5"/><path d="M7 8.5h10M12 3.5v10"/><path d="M4 20h16"/></svg>',
    pause:'<svg viewBox="0 0 24 24"><rect x="7" y="6" width="3.2" height="12" rx="1"/><rect x="14" y="6" width="3.2" height="12" rx="1"/></svg>',
    home:'<svg viewBox="0 0 24 24"><path d="M4 11l8-6 8 6"/><path d="M6 10v9h12v-9"/></svg>',
    check:'<svg viewBox="0 0 24 24"><polyline points="5 12 10 17 19 6"/></svg>',
    list:'<svg viewBox="0 0 24 24"><rect x="6" y="4" width="12" height="16" rx="2"/><rect x="9" y="2" width="6" height="4" rx="1"/><path d="M9 10h6M9 14h6"/></svg>',
    dispatch:'<svg viewBox="0 0 24 24"><path d="M12 3c-3.3 0-6 2.7-6 6 0 4 6 11 6 11s6-7 6-11c0-3.3-2.7-6-6-6z"/><circle cx="12" cy="9" r="2.2"/></svg>',
    camera:'<svg viewBox="0 0 24 24"><rect x="3" y="7" width="18" height="12" rx="2"/><circle cx="12" cy="13" r="3.2"/><path d="M8 7l1.5-3h5L16 7"/></svg>',
    camoff:'<svg viewBox="0 0 24 24"><rect x="3" y="7" width="18" height="12" rx="2"/><circle cx="12" cy="13" r="3.2"/><path d="M8 7l1.5-3h5L16 7"/><path d="M3 3l18 18"/></svg>',
    phone:'<svg viewBox="0 0 24 24"><path d="M5 4h3.5l1.5 4-2.2 1.6a12 12 0 0 0 5.6 5.6L16 13l4 1.5V18a2 2 0 0 1-2 2C10.8 20 4 13.2 4 6a2 2 0 0 1 1-2z"/></svg>',
    broadcast:'<svg viewBox="0 0 24 24"><circle cx="12" cy="12" r="2.2"/><path d="M7.5 7.5a6 6 0 0 0 0 9M16.5 7.5a6 6 0 0 1 0 9M5 5a9 9 0 0 0 0 14M19 5a9 9 0 0 1 0 14"/></svg>',
    alert:'<svg viewBox="0 0 24 24"><path d="M12 4l9 16H3z"/><path d="M12 10v4"/><circle cx="12" cy="17.5" r="0.6" fill="currentColor" stroke="none"/></svg>',
    back:'<svg viewBox="0 0 24 24"><polyline points="14 6 8 12 14 18"/></svg>',
    door:'<svg viewBox="0 0 24 24"><path d="M6 3h9v18H6z"/><circle cx="12" cy="12" r="1" fill="currentColor" stroke="none"/><path d="M6 21h12"/></svg>'
  };
  function paintIcons(root){(root||document).querySelectorAll('[data-ic]').forEach(function(e){if(!e.dataset.painted){e.innerHTML=ICONS[e.dataset.ic]||'';e.dataset.painted=1}})}

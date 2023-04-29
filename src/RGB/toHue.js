function rgb_to_hsv(r , g , b, rZeroRef, rOneRef, gZeroRef, gOneRef, bZeroRef, bOneRef) {
  rScale = rOneRef-rZeroRef;
  gScale = gOneRef-gZeroRef;
  bScale = bOneRef-bZeroRef;

  r = 1.0*(r-rZeroRef) / rScale;
  g = 1.0*(g-gZeroRef) / gScale;
  b = 1.0*(b-bZeroRef) / bScale;

  var cmax = Math.max(r, Math.max(g, b)); // maximum of r, g, b
  var cmin = Math.min(r, Math.min(g, b)); // minimum of r, g, b
  var diff = cmax - cmin; // diff of cmax and cmin.
  var h = -1; // s = -1; // h is hue value, s is saturation

  // if cmax and cmax are equal then h = 0
  if (cmax == cmin)
    h = 0;
  // if cmax equal r then compute h
  else if (cmax == r)
    h = (60 * ((g - b) / diff) + 360) % 360;
  // if cmax equal g then compute h
  else if (cmax == g)
   h = (60 * ((b - r) / diff) + 120) % 360;
  // if cmax equal b then compute h
  else if (cmax == b)
    h = (60 * ((r - g) / diff) + 240) % 360;
  return h;
}

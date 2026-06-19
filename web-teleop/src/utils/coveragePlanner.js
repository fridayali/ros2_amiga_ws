export function lineIntersection(x1, y1, x2, y2, x3, y3, x4, y4) {
  const d = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
  if (Math.abs(d) < 1e-10) return null;

  const t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / d;
  const u = -((x1 - x2) * (y1 - y3) - (y1 - y2) * (x1 - x3)) / d;
  const epsilon = 1e-9;

  if (t >= -epsilon && t <= 1 + epsilon && u >= -epsilon && u <= 1 + epsilon) {
    return { x: x1 + t * (x2 - x1), y: y1 + t * (y2 - y1) };
  }

  return null;
}

export function polygonIntersections(lineStart, lineEnd, polygon) {
  const raw = [];

  for (let i = 0; i < polygon.length; i += 1) {
    const pt = lineIntersection(
      lineStart.x,
      lineStart.y,
      lineEnd.x,
      lineEnd.y,
      polygon[i].x,
      polygon[i].y,
      polygon[(i + 1) % polygon.length].x,
      polygon[(i + 1) % polygon.length].y
    );
    if (pt) raw.push(pt);
  }

  const epsilon = 1e-6;
  const unique = [];

  for (const pt of raw) {
    if (!unique.some(p => Math.abs(p.x - pt.x) < epsilon && Math.abs(p.y - pt.y) < epsilon)) {
      unique.push(pt);
    }
  }

  if (Math.abs(lineEnd.x - lineStart.x) > Math.abs(lineEnd.y - lineStart.y)) {
    unique.sort((a, b) => a.x - b.x);
  } else {
    unique.sort((a, b) => a.y - b.y);
  }

  return unique;
}

export function orderedPolygonPoints(points, startCorner = 0) {
  if (points.length < 3) return points;

  const centerX = points.reduce((sum, p) => sum + p.x, 0) / points.length;
  const centerY = points.reduce((sum, p) => sum + p.y, 0) / points.length;
  const sorted = [...points].sort(
    (a, b) => Math.atan2(a.y - centerY, a.x - centerX) - Math.atan2(b.y - centerY, b.x - centerX)
  );
  const corner = Math.max(0, Math.min(sorted.length - 1, Number(startCorner) || 0));

  return Array.from({ length: sorted.length }, (_, i) => sorted[(i + corner) % sorted.length]);
}

export function generateCoveragePreviewPath(points, style, lineSpacing, sweepAngle, startCorner = 0) {
  if (points.length < 3) return [];

  const spacing = Number(lineSpacing);
  if (!Number.isFinite(spacing) || spacing <= 0) return [];

  const polygon = orderedPolygonPoints(points, startCorner);
  const minX = Math.min(...polygon.map(p => p.x));
  const maxX = Math.max(...polygon.map(p => p.x));
  const minY = Math.min(...polygon.map(p => p.y));
  const maxY = Math.max(...polygon.map(p => p.y));
  const centerX = polygon.reduce((sum, p) => sum + p.x, 0) / polygon.length;
  const centerY = polygon.reduce((sum, p) => sum + p.y, 0) / polygon.length;
  const pad = Math.max(maxX - minX, maxY - minY) * 2 + 2;
  const path = [];
  const hasLength = (a, b) => Math.abs(b.x - a.x) > 1e-5 || Math.abs(b.y - a.y) > 1e-5;

  const pushSweepSegment = (pointsOnLine, flip) => {
    if (pointsOnLine.length < 2) return false;
    const a = pointsOnLine[0];
    const b = pointsOnLine[pointsOnLine.length - 1];
    if (!hasLength(a, b)) return false;
    if (flip) {
      path.push(b, a);
    } else {
      path.push(a, b);
    }
    return true;
  };

  if (style === "ladder") {
    let flip = false;
    for (let x = minX + spacing * 0.5; x <= maxX + 1e-6; x += spacing) {
      const pts = polygonIntersections({ x, y: minY - pad }, { x, y: maxY + pad }, polygon);
      if (pushSweepSegment(pts, flip)) flip = !flip;
    }
  } else if (style === "diagonal") {
    const angleRad = ((Number(sweepAngle) || 0) % 360) * Math.PI / 180;
    const cA = Math.cos(angleRad);
    const sA = Math.sin(angleRad);
    const dL = Math.sqrt((maxX - minX) ** 2 + (maxY - minY) ** 2) * 1.5 + 1;
    let flip = false;

    for (let offset = -dL; offset <= dL + 1e-6; offset += spacing) {
      const pcx = centerX - offset * sA;
      const pcy = centerY + offset * cA;
      const pts = polygonIntersections(
        { x: pcx - dL * cA, y: pcy - dL * sA },
        { x: pcx + dL * cA, y: pcy + dL * sA },
        polygon
      );
      if (pushSweepSegment(pts, flip)) flip = !flip;
    }
  } else {
    let flip = false;
    for (let y = minY + spacing * 0.5; y <= maxY + 1e-6; y += spacing) {
      const pts = polygonIntersections({ x: minX - pad, y }, { x: maxX + pad, y }, polygon);
      if (pushSweepSegment(pts, flip)) flip = !flip;
    }
  }

  return path;
}

export function estimateCoverageLineCount(points, style, lineSpacing) {
  if (points.length < 3) return 0;

  const spacing = Number(lineSpacing);
  if (!Number.isFinite(spacing) || spacing <= 0) return 0;

  const xs = points.map(p => p.x);
  const ys = points.map(p => p.y);
  const width = Math.max(...xs) - Math.min(...xs);
  const height = Math.max(...ys) - Math.min(...ys);

  if (style === "ladder") return Math.max(0, Math.floor(width / spacing));
  if (style === "diagonal") return Math.max(0, Math.floor(Math.sqrt(width * width + height * height) / spacing));
  return Math.max(0, Math.floor(height / spacing));
}

export function latLngToLocalMeters(latLng, anchor) {
  if (!latLng || !anchor) return null;

  const earthR = 6378137;
  const latRad = anchor.lat * Math.PI / 180;
  const dLat = (latLng.lat - anchor.lat) * Math.PI / 180;
  const dLng = (latLng.lng - anchor.lng) * Math.PI / 180;

  return {
    x: dLng * earthR * Math.cos(latRad),
    y: dLat * earthR
  };
}

export function localMetersToLatLng(point, anchor) {
  if (!point || !anchor) return null;

  const earthR = 6378137;
  const latRad = anchor.lat * Math.PI / 180;

  return {
    lat: anchor.lat + point.y / earthR * 180 / Math.PI,
    lng: anchor.lng + point.x / (earthR * Math.cos(latRad)) * 180 / Math.PI
  };
}

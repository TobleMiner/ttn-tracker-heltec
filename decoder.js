function exp2(power) {
  return Math.pow(2, power);
}

function fix24_to_float(bytes) {
  var exp = exp2(15);
  var val24 = bytes[0] | (bytes[1] << 8) | (bytes[2] << 16);
  var res = (val24 >> 15) & 0xFF;
  res += (val24 & 0x7FFF) / exp;
  if((val24 & 0x800000) != 0) {
    res = -res;
  }
  return res;
}

function getFloat(bytes) {
  var bits = (bytes[0] << 24) | (bytes[1] << 16) | (bytes[2] << 8) | (bytes[3]);
  var sign = ((bits >>> 31) == 0) ? 1.0 : -1.0;
  var e = ((bits >>> 23) & 0xff);
  var m = (e == 0) ? (bits & 0x7fffff) << 1 : (bits & 0x7fffff) | 0x800000;
  var f = sign * m * exp2(e - 150);
  return f;
}

function decode_hdop(hdop) {
  var dec = (hdop >> 4) & 0xf;
  var frac = (hdop & 0xf) / 10;
  return dec + frac;
}

function decode_ufloat8(byte) {
  var mantissa = (byte & 0xf) | (1 << 4);
  var exponent = ((byte >> 4) & 0xf) - 2;
  if(exponent == 0) {
    return 1.0;
  }
  exponent -= 4;
  return mantissa * exp2(exponent);
}

function Decoder(bytes, port) {
  var lat = fix24_to_float(bytes.slice(0, 3));
  var latSuffix = lat < 0 ? "S" : "N";
  var lng = fix24_to_float(bytes.slice(3, 6));
  var lngSuffix = lng < 0 ? "W" : "E";
  var decoded = {
//    "latitude": Math.abs(lat).toString() + " " + latSuffix,
//    "longitude": Math.abs(lng).toString() + " " + lngSuffix,
    "latitude": lat.toString(),
    "longitude": lng.toString(),
    "hdop": decode_hdop(bytes[6]),
    "height": decode_ufloat8(bytes[7]),
  };

  return decoded;
}

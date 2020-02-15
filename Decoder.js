// Add this to your payload functions in the application console from TTN
// This piece of code parses the bytes (in decimal) to hex and then to ascii to read a string

function Decoder(bytes, port) {
  var hex = [];

  for(var i = 0; i < bytes.length; i++) {
    hex[i] = (bytes[i].toString(16));
  }

  var hex = hex.join("").toString();

  var str = '';
    for (var j = 0; (j < hex.length && hex.substr(j, 2) !== '00'); j += 2){
      str += String.fromCharCode(parseInt(hex.substr(j, 2), 16));
    }
  
  return {
    data: str
  };
}

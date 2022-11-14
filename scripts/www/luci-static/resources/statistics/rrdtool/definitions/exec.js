'use strict';
'require baseclass';
return baseclass.extend({

	title: _('Weather'),

	rrdargs: function(graph, host, plugin, plugin_instance, dtype) {

        var temperature = {
            title: "%H: Temperature",
            vlabel: "Celcius (°C)",
            number_format: "%5.1lf °C",
			data: {
                sources: {
                    temperature: ["value"],
                    temperature2: ["value"],
                    feelslike: ["value"]
                },
				options: {
					temperature: {
						title:         "Temperature (DHT22 Sensor)",
                        noarea: 		true,
                        overlay: 		true
					},
					temperature2: {
						title:         "Temperature (BMP180 Sensor)",
                        noarea: 		true,
                        overlay: 		true
					},
					feelslike: {
						title:         "Feels Like (DHT22 Sensor)",
                        noarea: 		true,
                        overlay: 		true
					}
				}
			}
        };

        var humidity = {
            title: "%H: Humidity",
            vlabel: "percentage (%)",
            number_format: "%5.1lf",
			data: {
                sources: {
                    humidity: ["value"]
                },
				options: {
					humidity: {
						title:         "Humidity",
                        noarea: 		true,
                        overlay: 		true
					}
				}
			}
        };

        var uv = {
            title: "%H: Ultraviolet Surface Energy",
            vlabel: "Energy (mW/cm²)",
            number_format: "%5.1lf",
			data: {
                sources: {
                    uv: ["value"]
                },
				options: {
					uv: {
						title:         "Energy",
                        noarea: 		true,
                        overlay: 		true
					}
				}
			}
        };

        var duv = {
            title: "%H: Ultraviolet Index",
            vlabel: "Index",
            number_format: "%5.1lf",
			data: {
                sources: {
                    duv: ["value"]
                },
				options: {
					duv: {
						title:         "Index",
                        noarea: 		true,
                        overlay: 		true
					}
				}
			}
        };

        var pressure = {
            title: "%H: Atmosphere Pressure",
            vlabel: "kilo Pascal (kPa)",
            number_format: "%5.1lf kPa",
			data: {
                sources: {
                    pressure: ["value"],
                    pressuresea: ["value"]
                },
				options: {
					pressure: {
						title:         "Station Pressure",
                        noarea: 		true,
                        overlay: 		true
					},
					pressuresea: {
						title:         "Barometric Sea Level Pressure",
                        noarea: 		true,
                        overlay: 		true
					}
				}
			}
        };

        var altitude = {
            title: "%H: Altitude",
            vlabel: "meters (m)",
            number_format: "%5.1lf m",
			data: {
                sources: {
                    altitudereal: ["value"]
                },
				options: {
					altitudereal: {
						title:         "Altitude",
                        noarea: 		true,
                        overlay: 		true
					}
				}
			}
        };

        return [temperature, humidity, uv, duv, pressure, altitude];

	}

});

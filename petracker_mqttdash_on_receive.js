var obj = JSON.parse(event.payload);
event.data["latitude"] = obj.latitude;
event.data["longitude"] = obj.longitude;

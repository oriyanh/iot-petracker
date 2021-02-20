var obj = JSON.parse(event.payload);
if (!event.data) {
    event.data ={};}


event.data["latitude"] = obj.latitude;
event.data["longitude"] = obj.longitude;
var url_obj ='https://www.google.com/maps/search/?api=1&query=';
var gmap_url = url_obj + event.data["latitude"] + "," + event.data["longitude"];
app.openUri(gmap_url);
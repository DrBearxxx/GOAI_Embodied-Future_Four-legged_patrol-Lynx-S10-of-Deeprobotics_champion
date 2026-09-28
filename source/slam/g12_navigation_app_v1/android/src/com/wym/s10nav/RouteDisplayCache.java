package com.wym.s10nav;

import org.json.*;
import java.util.Iterator;

/** Display geometry only. Never restore robot state, progress, tickets or previews. */
final class RouteDisplayCache {
    static JSONObject snapshot(String assetId,String endpoint,JSONObject mission) throws JSONException {
        JSONObject display=new JSONObject();
        display.put("route_id",mission.getString("route_id"));
        display.put("catalog",mission.getJSONObject("catalog"));
        display.put("calibration_revision",mission.optInt("calibration_revision"));
        display.put("presets_revision",mission.optInt("presets_revision"));
        return new JSONObject().put("schema",1).put("asset_id",assetId).put("endpoint",endpoint)
            .put("saved_at",System.currentTimeMillis()).put("mission",display);
    }

    static JSONObject restore(String raw,String assetId,String endpoint) {
        try {
            JSONObject cache=new JSONObject(raw);
            if(cache.optInt("schema")!=1||!assetId.equals(cache.optString("asset_id"))||!endpoint.equals(cache.optString("endpoint")))return null;
            JSONObject mission=cache.getJSONObject("mission"),catalog=mission.getJSONObject("catalog");
            if(catalog.optJSONObject(mission.getString("route_id"))==null)return null;
            for(Iterator<String> it=catalog.keys();it.hasNext();) {
                JSONObject route=catalog.getJSONObject(it.next());
                JSONArray points=route.getJSONArray("waypoints"),edges=route.getJSONArray("edges");
                if(points.length()==0||edges.length()!=points.length()-1)return null;
                for(int i=0;i<points.length();i++) {
                    JSONArray xyz=points.getJSONObject(i).getJSONArray("xyz");
                    if(xyz.length()!=3)return null;
                    for(int j=0;j<3;j++)if(!Double.isFinite(xyz.getDouble(j)))return null;
                }
            }
            // Apply the allowlist on read too, even if an older writer saved more fields.
            return snapshot(assetId,endpoint,mission).getJSONObject("mission");
        } catch(Exception e) {return null;}
    }
}

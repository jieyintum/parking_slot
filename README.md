# fusion_se_pse
------------------------------
topic:
	sub avpe seg topic: /cem/vision/avpe/postpro_segment_map
	sub avpe(slot) topic: /cem/vision/avpe/postpro_segs
	sub slot topic: /cem/vision/pld/detect_objects
	sub wall pillar topic: /cem/fusion/static_elements
	pub avpe topic: /cem/fusion/static_elements_avpe
	pub slot topic: /cem/tracking/pld_tracking_slots

-------------------------------
avpe class:
	GROUND_PILLAR = 22
    	GROUND_WALL = 23
    	GROUND_LANE_LINE = 24
    	GROUND_ZEBRA = 25
	GROUND_ARROW_STARGIHT = 26
	

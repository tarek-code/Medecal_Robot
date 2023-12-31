PULSONIX_LIBRARY_ASCII "SamacSys ECAD Model"
//643239/965719/2.49/14/4/Integrated Circuit

(asciiHeader
	(fileUnits MM)
)
(library Library_1
	(padStyleDef "r75_35"
		(holeDiam 0)
		(padShape (layerNumRef 1) (padShapeType Rect)  (shapeWidth 0.350) (shapeHeight 0.750))
		(padShape (layerNumRef 16) (padShapeType Ellipse)  (shapeWidth 0) (shapeHeight 0))
	)
	(textStyleDef "Normal"
		(font
			(fontType Stroke)
			(fontFace "Helvetica")
			(fontHeight 1.27)
			(strokeWidth 0.127)
		)
	)
	(patternDef "MAX30102EFDT" (originalName "MAX30102EFDT")
		(multiLayer
			(pad (padNum 1) (padStyleRef r75_35) (pt -1.200, 2.400) (rotation 90))
			(pad (padNum 2) (padStyleRef r75_35) (pt -1.200, 1.600) (rotation 90))
			(pad (padNum 3) (padStyleRef r75_35) (pt -1.200, 0.800) (rotation 90))
			(pad (padNum 4) (padStyleRef r75_35) (pt -1.200, 0.000) (rotation 90))
			(pad (padNum 5) (padStyleRef r75_35) (pt -1.200, -0.800) (rotation 90))
			(pad (padNum 6) (padStyleRef r75_35) (pt -1.200, -1.600) (rotation 90))
			(pad (padNum 7) (padStyleRef r75_35) (pt -1.200, -2.400) (rotation 90))
			(pad (padNum 8) (padStyleRef r75_35) (pt 1.200, -2.400) (rotation 90))
			(pad (padNum 9) (padStyleRef r75_35) (pt 1.200, -1.600) (rotation 90))
			(pad (padNum 10) (padStyleRef r75_35) (pt 1.200, -0.800) (rotation 90))
			(pad (padNum 11) (padStyleRef r75_35) (pt 1.200, 0.000) (rotation 90))
			(pad (padNum 12) (padStyleRef r75_35) (pt 1.200, 0.800) (rotation 90))
			(pad (padNum 13) (padStyleRef r75_35) (pt 1.200, 1.600) (rotation 90))
			(pad (padNum 14) (padStyleRef r75_35) (pt 1.200, 2.400) (rotation 90))
		)
		(layerContents (layerNumRef 18)
			(attr "RefDes" "RefDes" (pt -0.475, 0.000) (textStyleRef "Normal") (isVisible True))
		)
		(layerContents (layerNumRef 28)
			(line (pt -1.65 2.8) (pt 1.65 2.8) (width 0.025))
		)
		(layerContents (layerNumRef 28)
			(line (pt 1.65 2.8) (pt 1.65 -2.8) (width 0.025))
		)
		(layerContents (layerNumRef 28)
			(line (pt 1.65 -2.8) (pt -1.65 -2.8) (width 0.025))
		)
		(layerContents (layerNumRef 28)
			(line (pt -1.65 -2.8) (pt -1.65 2.8) (width 0.025))
		)
		(layerContents (layerNumRef Courtyard_Top)
			(line (pt -3.6 3.8) (pt 2.65 3.8) (width 0.1))
		)
		(layerContents (layerNumRef Courtyard_Top)
			(line (pt 2.65 3.8) (pt 2.65 -3.8) (width 0.1))
		)
		(layerContents (layerNumRef Courtyard_Top)
			(line (pt 2.65 -3.8) (pt -3.6 -3.8) (width 0.1))
		)
		(layerContents (layerNumRef Courtyard_Top)
			(line (pt -3.6 -3.8) (pt -3.6 3.8) (width 0.1))
		)
		(layerContents (layerNumRef 18)
			(line (pt -0.4 2.8) (pt 0.4 2.8) (width 0.1))
		)
		(layerContents (layerNumRef 18)
			(line (pt -0.4 -2.8) (pt 0.4 -2.8) (width 0.1))
		)
		(layerContents (layerNumRef 18)
			(line (pt -2.6 2.4) (pt -2.6 2.4) (width 0.2))
		)
		(layerContents (layerNumRef 18)
			(arc (pt -2.5, 2.4) (radius 0.1) (startAngle 180.0) (sweepAngle 180.0) (width 0.2))
		)
		(layerContents (layerNumRef 18)
			(line (pt -2.4 2.4) (pt -2.4 2.4) (width 0.2))
		)
		(layerContents (layerNumRef 18)
			(arc (pt -2.5, 2.4) (radius 0.1) (startAngle .0) (sweepAngle 180.0) (width 0.2))
		)
	)
	(symbolDef "MAX30102EFD+T" (originalName "MAX30102EFD+T")

		(pin (pinNum 1) (pt 0 mils 0 mils) (rotation 0) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 230 mils -25 mils) (rotation 0]) (justify "Left") (textStyleRef "Normal"))
		))
		(pin (pinNum 2) (pt 0 mils -100 mils) (rotation 0) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 230 mils -125 mils) (rotation 0]) (justify "Left") (textStyleRef "Normal"))
		))
		(pin (pinNum 3) (pt 0 mils -200 mils) (rotation 0) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 230 mils -225 mils) (rotation 0]) (justify "Left") (textStyleRef "Normal"))
		))
		(pin (pinNum 4) (pt 0 mils -300 mils) (rotation 0) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 230 mils -325 mils) (rotation 0]) (justify "Left") (textStyleRef "Normal"))
		))
		(pin (pinNum 5) (pt 0 mils -400 mils) (rotation 0) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 230 mils -425 mils) (rotation 0]) (justify "Left") (textStyleRef "Normal"))
		))
		(pin (pinNum 6) (pt 0 mils -500 mils) (rotation 0) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 230 mils -525 mils) (rotation 0]) (justify "Left") (textStyleRef "Normal"))
		))
		(pin (pinNum 7) (pt 0 mils -600 mils) (rotation 0) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 230 mils -625 mils) (rotation 0]) (justify "Left") (textStyleRef "Normal"))
		))
		(pin (pinNum 8) (pt 1400 mils -600 mils) (rotation 180) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 1170 mils -625 mils) (rotation 0]) (justify "Right") (textStyleRef "Normal"))
		))
		(pin (pinNum 9) (pt 1400 mils -500 mils) (rotation 180) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 1170 mils -525 mils) (rotation 0]) (justify "Right") (textStyleRef "Normal"))
		))
		(pin (pinNum 10) (pt 1400 mils -400 mils) (rotation 180) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 1170 mils -425 mils) (rotation 0]) (justify "Right") (textStyleRef "Normal"))
		))
		(pin (pinNum 11) (pt 1400 mils -300 mils) (rotation 180) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 1170 mils -325 mils) (rotation 0]) (justify "Right") (textStyleRef "Normal"))
		))
		(pin (pinNum 12) (pt 1400 mils -200 mils) (rotation 180) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 1170 mils -225 mils) (rotation 0]) (justify "Right") (textStyleRef "Normal"))
		))
		(pin (pinNum 13) (pt 1400 mils -100 mils) (rotation 180) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 1170 mils -125 mils) (rotation 0]) (justify "Right") (textStyleRef "Normal"))
		))
		(pin (pinNum 14) (pt 1400 mils 0 mils) (rotation 180) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 1170 mils -25 mils) (rotation 0]) (justify "Right") (textStyleRef "Normal"))
		))
		(line (pt 200 mils 100 mils) (pt 1200 mils 100 mils) (width 6 mils))
		(line (pt 1200 mils 100 mils) (pt 1200 mils -700 mils) (width 6 mils))
		(line (pt 1200 mils -700 mils) (pt 200 mils -700 mils) (width 6 mils))
		(line (pt 200 mils -700 mils) (pt 200 mils 100 mils) (width 6 mils))
		(attr "RefDes" "RefDes" (pt 1250 mils 300 mils) (justify Left) (isVisible True) (textStyleRef "Normal"))
		(attr "Type" "Type" (pt 1250 mils 200 mils) (justify Left) (isVisible True) (textStyleRef "Normal"))

	)
	(compDef "MAX30102EFD+T" (originalName "MAX30102EFD+T") (compHeader (numPins 14) (numParts 1) (refDesPrefix IC)
		)
		(compPin "1" (pinName "N.C._1") (partNum 1) (symPinNum 1) (gateEq 0) (pinEq 0) (pinType Unknown))
		(compPin "2" (pinName "SCL") (partNum 1) (symPinNum 2) (gateEq 0) (pinEq 0) (pinType Unknown))
		(compPin "3" (pinName "SDA") (partNum 1) (symPinNum 3) (gateEq 0) (pinEq 0) (pinType Unknown))
		(compPin "4" (pinName "PGND") (partNum 1) (symPinNum 4) (gateEq 0) (pinEq 0) (pinType Unknown))
		(compPin "5" (pinName "N.C._2") (partNum 1) (symPinNum 5) (gateEq 0) (pinEq 0) (pinType Unknown))
		(compPin "6" (pinName "N.C._3") (partNum 1) (symPinNum 6) (gateEq 0) (pinEq 0) (pinType Unknown))
		(compPin "7" (pinName "N.C._4") (partNum 1) (symPinNum 7) (gateEq 0) (pinEq 0) (pinType Unknown))
		(compPin "8" (pinName "N.C._5") (partNum 1) (symPinNum 8) (gateEq 0) (pinEq 0) (pinType Unknown))
		(compPin "9" (pinName "VLED+_1") (partNum 1) (symPinNum 9) (gateEq 0) (pinEq 0) (pinType Unknown))
		(compPin "10" (pinName "VLED+_2") (partNum 1) (symPinNum 10) (gateEq 0) (pinEq 0) (pinType Unknown))
		(compPin "11" (pinName "VDD") (partNum 1) (symPinNum 11) (gateEq 0) (pinEq 0) (pinType Unknown))
		(compPin "12" (pinName "GND") (partNum 1) (symPinNum 12) (gateEq 0) (pinEq 0) (pinType Unknown))
		(compPin "13" (pinName "__INT") (partNum 1) (symPinNum 13) (gateEq 0) (pinEq 0) (pinType Unknown))
		(compPin "14" (pinName "N.C._6") (partNum 1) (symPinNum 14) (gateEq 0) (pinEq 0) (pinType Unknown))
		(attachedSymbol (partNum 1) (altType Normal) (symbolName "MAX30102EFD+T"))
		(attachedPattern (patternNum 1) (patternName "MAX30102EFDT")
			(numPads 14)
			(padPinMap
				(padNum 1) (compPinRef "1")
				(padNum 2) (compPinRef "2")
				(padNum 3) (compPinRef "3")
				(padNum 4) (compPinRef "4")
				(padNum 5) (compPinRef "5")
				(padNum 6) (compPinRef "6")
				(padNum 7) (compPinRef "7")
				(padNum 8) (compPinRef "8")
				(padNum 9) (compPinRef "9")
				(padNum 10) (compPinRef "10")
				(padNum 11) (compPinRef "11")
				(padNum 12) (compPinRef "12")
				(padNum 13) (compPinRef "13")
				(padNum 14) (compPinRef "14")
			)
		)
		(attr "Manufacturer_Name" "Maxim Integrated")
		(attr "Manufacturer_Part_Number" "MAX30102EFD+T")
		(attr "Mouser Part Number" "700-MAX30102EFD+T")
		(attr "Mouser Price/Stock" "https://www.mouser.co.uk/ProductDetail/Maxim-Integrated/MAX30102EFD%2bT?qs=nVS1qgv%252BQrkHA4%2FoFYriFA%3D%3D")
		(attr "Arrow Part Number" "MAX30102EFD+T")
		(attr "Arrow Price/Stock" "https://www.arrow.com/en/products/max30102efdt/maxim-integrated?region=nac")
		(attr "Mouser Testing Part Number" "")
		(attr "Mouser Testing Price/Stock" "")
		(attr "Description" "Biometric Sensors Integrated Optical Sensor")
		(attr "<Hyperlink>" "https://datasheets.maximintegrated.com/en/ds/MAX30102.pdf")
		(attr "<Component Height>" "1.65")
		(attr "<STEP Filename>" "MAX30102EFD+T.stp")
		(attr "<STEP Offsets>" "X=0;Y=0;Z=0")
		(attr "<STEP Rotation>" "X=0;Y=0;Z=0")
	)

)

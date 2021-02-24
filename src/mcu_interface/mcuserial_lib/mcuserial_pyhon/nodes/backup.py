ThrEvent = threading.Event()
            waThread = threading.Thread(target=sendMessage, args=(ThrEvent, mcuSerialInterface, "Thread A"))
            waThread.daemon = True
            waThread.start()


            wbThread = threading.Thread(target=sendMessage, args=(ThrEvent, mcuSerialInterface, "Thread B"))
            wbThread.daemon = True
            wbThread.start()

            wcThread = threading.Thread(target=sendMessage, args=(ThrEvent, mcuSerialInterface, "Thread C"))
            wcThread.daemon = True
            wcThread.start()

            wdThread = threading.Thread(target=sendMessage, args=(ThrEvent, mcuSerialInterface, "Thread D"))
            wdThread.daemon = True
            wdThread.start()

            weThread = threading.Thread(target=sendMessage, args=(ThrEvent, mcuSerialInterface, "Thread E"))
            weThread.daemon = True
            weThread.start()


            weThread = threading.Thread(target=sendMessage, args=(ThrEvent, mcuSerialInterface, "Thread F"))
            weThread.daemon = True
            weThread.start()



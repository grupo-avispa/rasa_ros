# Contains information about an entity extracted from a parsed message

uint16 start  # Start index of the entity in the text
uint16 end  # End index of the entity in the text
string value  # Value of the entity
string entity  # Type of the entity
float32 confidence  # Confidence of the entity
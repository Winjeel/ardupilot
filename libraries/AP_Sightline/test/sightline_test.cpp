

#define INCLUDE_PRINT (1)

#include "../SL_MsgBuffer.h"
#include <iostream>
#include <cstring>


void test_SlidingWindow(void) {
	CircularBuffer<16, '!'> sw;

	uint8_t arr[] = {'0', '1', '2', '3', '4', '5', '6', '7',
			         '8', '9', 'A', 'B', 'C', 'D', 'E', 'F', };

	uint8_t out[33] = { 0, };
	size_t pushed;

	std::cout << "sliding 5" << std::endl;
	sw.removeFromHead(5);
	sw.printChars();

	std::cout << "pushing 3" << std::endl;
	sw.push(arr, 3);
	sw.printChars();

	std::cout << "pushing 1" << std::endl;
	sw.push('X');
	sw.printChars();

	std::cout << "pushing 1" << std::endl;
	sw.push('Y');
	sw.printChars();

	std::cout << "extract 3" << std::endl;
	sw.extract(out, 3);
	std::cout << "  = " << out << std::endl;
	sw.printChars();

	std::cout << "extract 3 @ 2" << std::endl;
	sw.extract(out, 3, 2);
	std::cout << "  = " << out << std::endl;
	sw.printChars();

	std::cout << "extract 8" << std::endl;
	sw.extract(out, 8);
	std::cout << "  = " << out << std::endl;
	sw.printChars();

	std::cout << "extract 8 @ 2" << std::endl;
	sw.extract(out, 8, 2);
	std::cout << "  = " << out << std::endl;
	sw.printChars();

	std::cout << "extract 8 @ 10" << std::endl;
	sw.extract(out, 8, 10);
	std::cout << "  = " << out << std::endl;
	sw.printChars();

	std::cout << "extract 32" << std::endl;
	sw.extract(out, 32);
	std::cout << "  = " << out << std::endl;
	sw.printChars();

	std::cout << "extract 32 @ 10" << std::endl;
	sw.extract(out, 32, 10);
	std::cout << "  = " << out << std::endl;
	sw.printChars();

	std::cout << "sliding 0" << std::endl;
	sw.removeFromHead(0);
	sw.printChars();

	std::cout << "sliding 2" << std::endl;
	sw.removeFromHead(2);
	sw.printChars();

	std::cout << "sliding 2" << std::endl;
	sw.removeFromHead(2);
	sw.printChars();

	std::cout << "sliding 20" << std::endl;
	sw.removeFromHead(20);
	sw.printChars();

	std::cout << "sliding 2" << std::endl;
	sw.removeFromHead(2);
	sw.printChars();

	std::cout << "pushing 8" << std::endl;
	std::cout << "  = " << sw.push(arr, 8) << std::endl;
	sw.printChars();

	std::cout << "pushing 8" << std::endl;
	std::cout << "  = " << sw.push(&arr[8], 8) << std::endl;
	sw.printChars();

	std::cout << "pushing 1" << std::endl;

	pushed = sw.push('Z');
	std::cout << "  = " << pushed << std::endl;
	sw.printChars();

	std::cout << "pushing 1" << std::endl;
	pushed = sw.push(arr, 1);
	std::cout << "  = " << pushed << std::endl;
	sw.printChars();

	std::cout << "pushing 3" << std::endl;
	pushed = sw.push(arr, 3);
	std::cout << "  = " << pushed << std::endl;
	sw.printChars();

	sw.removeFromTail(0);
	sw.printChars();

	sw.removeFromTail(2);
	sw.printChars();

	sw.removeFromTail(2);
	sw.printChars();
}

void test_SL_MsgBuffer(void) {

	SL_MsgBuffer buff;
	SL_MsgId type;
	size_t received, consumed;


	uint8_t msg0[] = { SL_MAGIC_1, SL_MAGIC_2, 3, 0x28, 0x00, 0x73, };
	uint8_t msg1[] = { SL_MAGIC_1, SL_MAGIC_2, 3, 0x28, 0x01, 0x2d, };
	uint8_t msg2[] = { SL_MAGIC_1, SL_MAGIC_2, 3, 0x28, 0x02, 0xcf, };
	uint8_t msg3[] = { SL_MAGIC_1, SL_MAGIC_2, 3, 0x28, 0x0c, 0xd0, };

	uint8_t out[64] = { 0, };

	if (1) {
		buff.push(msg0, sizeof(msg0));
		buff.push(msg1, sizeof(msg1));
		buff.push(msg2, sizeof(msg2));
		buff.push(msg3, sizeof(msg3));
		buff.print("");

		// check lengths
		std::cout << "type = " << buff.assess() << std::endl;
		std::cout << "len = " << buff.getMsgLength() << std::endl;
		std::cout << "dataLen = " << buff.getMsgDataLength() << std::endl << std::endl;

		// check CRCs
		buff.copyMsg(out, 6);
		std::cout << "msg" << out[4] << "crc = " << out[5] << " ok=" << (out[5] == msg0[5]) << std::endl;
		buff.consumeMsg();
		buff.print("");

		buff.copyMsg(out, 6);
		std::cout << "msg" << out[4] << "crc = " << out[5] << " ok=" << (out[5] == msg1[5]) << std::endl;
		buff.consumeMsg();
		buff.print("");

		buff.copyMsg(out, 6);
		std::cout << "msg" << out[4] << "crc = " << out[5] << " ok=" << (out[5] == msg2[5]) << std::endl;
		buff.consumeMsg();
		buff.print("");

		buff.copyMsg(out, 6);
		std::cout << "msg" << out[4] << "crc = " << out[5] << " ok=" << (out[5] == msg3[5]) << std::endl;
		buff.consumeMsg();
		buff.print("");

		return;
	}

	if (0) {
		buff.push('!');
		buff.push('!');
		buff.push(msg0, sizeof(msg0));
		buff.push(msg1, sizeof(msg1));

		buff.print("");
		type = buff.assess();
		std::cout << "type = " << type << std::endl << std::endl;
		buff.consumeMsg();

		buff.print("");
		type = buff.assess();
		std::cout << "type = " << type << std::endl << std::endl;
		buff.consumeMsg();

		buff.print("");

		return;
	}


if (0) {
	for (uint8_t j = '1'; j <= '4'; j++) {
		for (size_t i = 0; i < sizeof(msg1) && buff.getBytesFree(); i++) {
			std::cout << j << "-M1 pushing = " << msg1[i] << std::endl;
			buff.push(msg1[i]);
			type = buff.assess();
			std::cout << "type = " << type << std::endl << std::endl;
		}
	}
	std::cout << "######################################" << std::endl << std::endl;

	std::cout << std::endl;
	type = buff.assess();
	std::cout << "type = " << type << std::endl << std::endl;
	buff.print("");
	consumed = buff.consumeMsg();
	std::cout << "consume " << consumed << std::endl;
	buff.print("");

	std::cout << std::endl;
	type = buff.assess();
	std::cout << "type = " << type << std::endl << std::endl;
	buff.print("");
	consumed = buff.consumeMsg();
	std::cout << "consume " << consumed << std::endl;
	buff.print("");

	return;
}


	for (size_t i = 0; i < sizeof(msg1) && buff.getBytesFree(); i++) {
		std::cout << "M1 pushing = " << msg1[i] << std::endl;
		buff.push(msg1[i]);
		SL_MsgId type = buff.assess();
		std::cout << "type = " << type << std::endl << std::endl;
	}

	memset(out, 0, sizeof(out));
	received = buff.copyData(out, buff.getMsgDataLength());
	std::cout << "1. out[" << received << "] = " << out << std::endl;

	for (size_t i = 0; i < sizeof(msg2) && buff.getBytesFree(); i++) {
		std::cout << "M2 pushing = " << msg2[i] << std::endl;
		buff.push(msg2[i]);
		type = buff.assess();
		std::cout << "type = " << type << std::endl << std::endl;
	}

	memset(out, 0, sizeof(out));
	received = buff.copyData(out, buff.getMsgDataLength());
	std::cout << "2. out[" << received << "] = " << out << std::endl;

	type = buff.assess();
	std::cout << "type = " << type << std::endl << std::endl;
	buff.print("");

	consumed = buff.consumeMsg();
	std::cout << "consume " << consumed << std::endl;
	type = buff.assess();
	std::cout << "type = " << type << std::endl << std::endl;
	buff.print("");

	memset(out, 0, sizeof(out));
	received = buff.copyData(out, buff.getMsgDataLength());
	std::cout << "3. out[" << received << "] = " << out << std::endl;
	buff.print("");

	consumed = buff.consumeMsg();
	std::cout << "consume " << consumed << std::endl;
	type = buff.assess();
	std::cout << "type = " << type << std::endl << std::endl;
	buff.print("");

	for (size_t i = 0; i < sizeof(msg3) && buff.getBytesFree(); i++) {
		std::cout << "M3 pushing = " << msg3[i] << std::endl;
		buff.push(msg3[i]);
		type = buff.assess();
		std::cout << "type = " << type << std::endl << std::endl;
	}

	type = buff.assess();
	std::cout << "type = " << type << std::endl << std::endl;
	buff.print("");

	memset(out, 0, sizeof(out));
	received = buff.copyData(out, buff.getMsgDataLength());
	std::cout << "4. out[" << received << "] = " << out << std::endl;
	buff.print("");

	std::cout << "consume " << buff.consumeMsg() << std::endl;
	type = buff.assess();
	std::cout << "type = " << type << std::endl << std::endl;
	buff.print("");
}




int main(void) {
	// TODO: turn these into proper test cases
	//test_SlidingWindow();
	test_SL_MsgBuffer();
}

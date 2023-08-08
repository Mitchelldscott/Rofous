

void init_process_hid() {
	HidProcessParams params;
	params.key = "";
	params.proc_id = report.get(2);
	params.key += char(report.get(3));
	params.key += char(report.get(4));
	params.key += char(report.get(5));
	params.n_config = report.get(6);
	params.n_inputs = report.get(7);

	for (int i = 0; i < n_inputs; i++) {
		params.input_ids.push(report.get(i + 8));
	}
	// add(key, id, n_config, n_inputs, input_ids);
	// push to new node queue
	hid_process_queue.push(params);
}

void config_process_hid() {
	HidProcessConfig new_config;
	new_config.proc_id = report.get(2);
	new_config.chunk_num = report.get(3);
	new_config.chunk_size = report.get(4);
	for (int i = 0; i < chunk_size; i++) {
		new_config.config.push(report.get_float((4 * i) + 5));
	}
	// Serial.printf("configure %i, %i, %i\n",id, chunk_num, chunk_size);
	// push config packet to config queue
	hid_config_queue.push(new_config);

}
void SynCor::dump_vector(Vector<float>* data) {
	report.put(3, data->size());
	for (int i = 0; i < data->size(); i++){
		report.put_float((4 * i) + 4, (*data)[i]);
	}
}

void push_hid() {
	switch (report.read()) {
		case 64:
			blink();										// only blink when connected to hid
			// Serial.printf("[%f]Report %i, %i, %i\n", lifetime, report.get(0), report.get(1), report.get(2));
			switch (report.get(0)) {
				case 255:
					// report.print();
					switch (report.get(1)) {
						case 1:
							init_process_hid();
							break;

						case 2:
							config_process_hid();
							break;

						default:
							break;
					}
					break;

				case 1:
					switch (report.get(1)) {
						case 1:
							if (hid_context_buffer.len() > report.get(2)) {
								dump_vector(hid_context_buffer[report.get(2)]->info());
							}
							break;

						case 2:
							if (hid_output_buffer.len() > report.get(2)) {
								dump_vector(hid_output_buffer[report.get(2)]->info());
							}
							break;

						default:
							break;
					}
					break;

				default:
					break;
			}
			break;
		
		default:
			// Serial.println("No report");
			break;
	}

	lifetime += hid_timers.secs(0, 1E3);
	hid_timers.set(0);
	report.put_float(60, lifetime);
	report.write();
}
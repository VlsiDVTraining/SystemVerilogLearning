program noff();
  initial begin
    mailbox mbx1, mbx2;
    
    mbx1 = new(); //Construct it
    mbx2 = new();
    
    fork
      begin //Producer for Mailbox 1
        repeat(10) begin
          int val;
          
          #50ns;
          std::randomize(val) with {val inside {[10:20]};};
          mbx1.put(val);
          //$display("Posted Item = %0d at %0t to Mailbox 1", val, $time);
        end
        //$display("Completed Producing 10 Items and Dropped to Mailbox 1 at %0t", $time);
      end
      begin //Consumer for Mailbox 1
        #10000ns;
        repeat(10) begin
          int val;
          
          mbx1.get(val);
          //$display("Received Item = %0d at %0t to Mailbox 1", val, $time);
          #10ns;
        end
        //$display("Completed Consuming 10 Items from Mailbox 1 at %0t", $time);
      end
      begin //Producer for Mailbox 2
        repeat(10) begin
          int val;
          int dly;
          
          std::randomize(dly) with {dly inside {[100:200]};};
          #(dly * 1ns);
          std::randomize(val) with {val inside {[100:200]};};
          mbx2.put(val);
          $display("Posted Item = %0d at %0t to Mailbox 2", val, $time);
        end
        $display("Completed Producing 10 Items and Dropped to Mailbox 2 at %0t", $time);
      end
      begin //Consumer for Mailbox 2
        $display("Starting to consume Items from Mailbox 2 at %0t", $time);
        repeat(10) begin
          int val;
          
          mbx2.get(val);
          $display("Received Item = %0d at %0t to Mailbox 2", val, $time);
        end
        $display("Completed Consuming 10 Items from Mailbox 2 at %0t", $time);
      end
    join
  end
endprogram

//=====================================================================================
//  UART Simple Verification Environment 
//=====================================================================================

class transaction;
  typedef enum bit {WRITE = 1'b0, READ = 1'b1} oper_type;

  randc oper_type oper;      // cyclic randomization of read/write
  rand bit [7:0]  dintx;     // data to be transmitted / expected on receive

  bit       rx;
  bit       tx;
  bit       newd;
  bit       donetx;
  bit       donerx;
  bit [7:0] doutrx;

  function transaction copy();
    copy = new();
    copy.oper    = this.oper;
    copy.dintx   = this.dintx;
    copy.rx      = this.rx;
    copy.tx      = this.tx;
    copy.newd    = this.newd;
    copy.donetx  = this.donetx;
    copy.donerx  = this.donerx;
    copy.doutrx  = this.doutrx;
  endfunction
endclass

//=====================================================================================
class generator;
  transaction                tr;
  mailbox #(transaction)     mbx;
  event                      done;
  int                        count = 0;
  event                      drvnext;
  event                      sconext;

  function new(mailbox #(transaction) mbx);
    this.mbx = mbx;
    tr = new();
  endfunction

  task run();
    repeat(count) begin
      assert(tr.randomize()) else $fatal(1, "[GEN] Randomization failed");
      mbx.put(tr.copy());
      $display("[%0t] [GEN] Oper = %s, Data = %0d", $time,
               (tr.oper == transaction::WRITE) ? "WRITE" : "READ", tr.dintx);

      @(drvnext);
      @(sconext);
    end
    ->done;
  endtask
endclass

//=====================================================================================
class driver;
  virtual uart_if            vif;
  transaction                tr;
  mailbox #(transaction)     mbx;     // gen → drv
  mailbox #(bit[7:0])        mbxds;   // drv → scoreboard (expected data)
  event                      drvnext;

  function new(mailbox #(bit[7:0]) mbxds, mailbox #(transaction) mbx);
    this.mbxds = mbxds;
    this.mbx   = mbx;
  endfunction

  task reset();
    vif.rst   <= 1;
    vif.rx    <= 1;
    vif.newd  <= 0;
    vif.dintx <= '0;
    repeat(10) @(posedge vif.uclktx);
    vif.rst   <= 0;
    $display("[%0t] [DRV] Reset done", $time);
  endtask

  task run();
    forever begin
      mbx.get(tr);

      if(tr.oper == transaction::WRITE) begin                // ---------- TX ----------
        @(posedge vif.uclktx);
        vif.newd  <= 1;
        vif.rx    <= 1;
        vif.dintx <= tr.dintx;
        @(posedge vif.uclktx);
        vif.newd  <= 0;

        mbxds.put(tr.dintx);                                 // expected data to SB
        $display("[%0t] [DRV] TX → Data = %0d", $time, tr.dintx);

        wait(vif.donetx == 1'b1);
        ->drvnext;
      end
      else begin                                              // ---------- RX ----------
        bit [7:0] data_to_send = tr.dintx;   // data we will drive on rx line

        @(posedge vif.uclkrx);
        vif.rx <= 0;                         // start bit

        for(int i = 0; i < 8; i++) begin
          @(posedge vif.uclkrx);
          vif.rx <= data_to_send[i];        // LSB first
        end

        @(posedge vif.uclkrx);
        vif.rx <= 1;                         // stop bit

        mbxds.put(tr.dintx);                 // expected data to scoreboard
        $display("[%0t] [DRV] RX driving data = %0d", $time, tr.dintx);

        wait(vif.donerx == 1'b1);
        ->drvnext;
      end
    end
  endtask
endclass

//=====================================================================================
class monitor;
  virtual uart_if            vif;
  mailbox #(bit[7:0])        mbx;
  bit [7:0]                  srx;   // captured TX data
  bit [7:0]                  rrx;   // captured RX data

  function new(mailbox #(bit[7:0]) mbx);
    this.mbx = mbx;
  endfunction

  task run();
    fork
      // --------------------- Monitor TX (write) ---------------------
      forever begin
        @(posedge vif.uclktx);
        if(vif.newd === 1'b1) begin
          // skip the cycle where newd is high
          @(posedge vif.uclktx);
          // start bit is now on tx, skip it
          @(posedge vif.uclktx);
          for(int i = 0; i < 8; i++) begin
            @(posedge vif.uclktx);
            srx[i] = vif.tx;
          end
          $display("[%0t] [MON] Captured TX data = %0d", $time, srx);
          mbx.put(srx);
        end
      end

      // --------------------- Monitor RX (read) ---------------------
      forever begin
        @(posedge vif.donerx);          // exactly when reception finishes
        #1;                             // tiny delay for doutrx to settle
        rrx = vif.doutrx;
        $display("[%0t] [MON] Captured RX data = %0d", $time, rrx);
        mbx.put(rrx);
      end
    join
  endtask
endclass

//=====================================================================================
class scoreboard;
  mailbox #(bit[7:0]) mbxds;   // driver   → scoreboard (expected)
  mailbox #(bit[7:0]) mbxms;   // monitor  → scoreboard (actual)
  event               sconext;

  function new(mailbox #(bit[7:0]) mbxds, mailbox #(bit[7:0]) mbxms);
    this.mbxds = mbxds;
    this.mbxms = mbxms;
  endfunction

  task run();
    bit [7:0] exp, act;
    forever begin
      mbxds.get(exp);
      mbxms.get(act);

      $display("[%0t] [SCO] Expected = %0d  |  Actual = %0d", $time, exp, act);
      if(exp === act)
        $display("   [PASS]");
      else
        $display("   [FAIL]");
      ->sconext;
    end
  endtask
endclass

//=====================================================================================
class environment;
  generator   gen;
  driver      drv;
  monitor     mon;
  scoreboard  sco;

  mailbox #(transaction) gd_mbx;
  mailbox #(bit[7:0])    ds_mbx, ms_mbx;

  event nextgd, nextgs;

  virtual uart_if vif;

  function new(virtual uart_if vif);
    this.vif = vif;

    gd_mbx = new();
    ds_mbx = new();
    ms_mbx = new();

    gen = new(gd_mbx);
    drv = new(ds_mbx, gd_mbx);
    mon = new(ms_mbx);
    sco = new(ds_mbx, ms_mbx);

    drv.vif = vif;
    mon.vif = vif;

    // connect events
    gen.drvnext = nextgd;
    drv.drvnext = nextgd;
    gen.sconext = nextgs;
    sco.sconext = nextgs;
  endfunction

  task pre_test();  drv.reset(); endtask
  task post_test();
    wait(gen.done.triggered);
    #200;
    $display("[%0t] Simulation finished cleanly!", $time);
  endtask

  task test();
    fork
      gen.run();
      drv.run();
      mon.run();
      sco.run();
    join_any
  endtask

  task run();
    pre_test();
    test();
    post_test();
  endtask
endclass

//=====================================================================================
/*interface uart_if;
  logic clk, rst;
  logic rx, tx;
  logic newd;
  logic [7:0] dintx;
  logic donetx, donerx;
  logic [7:0] doutrx;

  logic uclktx, uclkrx;   // internal baud clocks from DUT
endinterface
*/
//=====================================================================================
module tb;
  uart_if vif();

  uart_top #(1000000,9600) dut (
    .clk    (vif.clk),
    .rst    (vif.rst),
    .rx     (vif.rx),
    .newd   (vif.newd),
    .dintx  (vif.dintx),
    .tx     (vif.tx),
    .donetx (vif.donetx),
    .doutrx (vif.doutrx),
    .donerx (vif.donerx)
  );

  // Clock generation
  initial vif.clk = 0;
  always #10 vif.clk = ~vif.clk;

  // Connect internal UART clocks
  assign vif.uclktx = dut.utx.uclk;
  assign vif.uclkrx = dut.rtx.uclk;

  environment env;

  initial begin
    env = new(vif);
    env.gen.count = 10;      // 10 random transactions
    env.run();
    $finish;
  end

  initial begin
    $dumpfile("uart_tb.vcd");
    $dumpvars(0, tb);
  end
endmodule
